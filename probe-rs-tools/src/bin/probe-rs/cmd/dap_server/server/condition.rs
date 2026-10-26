//! The condition, hit count and log message of a breakpoint (the DAP `condition`,
//! `hitCondition` and `logMessage`).
//!
//! The server checks them when a core halts at the breakpoint, before it tells the client:
//! a halt that does not meet them is not reported, and the core runs again. The names in a
//! condition or a log message are read with the server's `evaluate`, so a name is a variable or
//! a register that `evaluate` knows; there are no operators inside a name.
//!
//! - A condition is `<name> <op> <value>` with `==`, `!=`, `<`, `<=`, `>`, `>=`. Several are
//!   joined with `&&` and `||`, without parentheses: `&&` binds first.
//! - A value is decimal, hexadecimal (`0x`), a character (`'a'`) or `true`/`false`. Both sides
//!   are compared as integers when both read as one, as numbers when both read as one, and as
//!   text otherwise.
//! - A hit count is `N` or `== N` (the Nth hit only), `>= N`, `> N` or `% N` (every Nth hit).
//!   Only the hits that meet the condition count.
//! - A log message is text with `{name}` replaced by the value of `name`. The core does not
//!   stop at a breakpoint that has one.

use std::cmp::Ordering;

/// A comparison operator of a condition.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Op {
    Eq,
    Ne,
    Lt,
    Le,
    Gt,
    Ge,
}

impl Op {
    /// The operators, longest first, so that `<=` is not read as `<`.
    const ALL: [(&'static str, Op); 6] = [
        ("==", Op::Eq),
        ("!=", Op::Ne),
        ("<=", Op::Le),
        (">=", Op::Ge),
        ("<", Op::Lt),
        (">", Op::Gt),
    ];

    fn holds(self, ordering: Ordering) -> bool {
        match self {
            Op::Eq => ordering == Ordering::Equal,
            Op::Ne => ordering != Ordering::Equal,
            Op::Lt => ordering == Ordering::Less,
            Op::Le => ordering != Ordering::Greater,
            Op::Gt => ordering == Ordering::Greater,
            Op::Ge => ordering != Ordering::Less,
        }
    }
}

/// `<name> <op> <value>`.
#[derive(Clone, Debug, PartialEq)]
struct Comparison {
    name: String,
    op: Op,
    value: String,
}

impl Comparison {
    fn parse(text: &str) -> Result<Self, String> {
        let text = text.trim();
        // The first operator in the text, at a place the longest (they come first); the `>` of
        // `->` (a pointer's member) is none.
        let bytes = text.as_bytes();
        // At the characters only: slicing inside one (a name in Korean) would panic.
        let Some((at, symbol, op)) = text.char_indices().find_map(|(at, _)| {
            if at > 0 && bytes[at] == b'>' && bytes[at - 1] == b'-' {
                return None;
            }
            Op::ALL
                .iter()
                .find(|(symbol, _)| text[at..].starts_with(symbol))
                .map(|(symbol, op)| (at, *symbol, *op))
        }) else {
            return Err(format!(
                "`{text}` has no comparison: write `<name> <op> <value>` with ==, !=, <, <=, > or >="
            ));
        };
        let name = text[..at].trim();
        let value = text[at + symbol.len()..].trim();
        if name.is_empty() || name.contains(char::is_whitespace) {
            return Err(format!(
                "`{text}` needs one variable or register name before {symbol}"
            ));
        }
        if value.is_empty() {
            return Err(format!("`{text}` needs a value after {symbol}"));
        }
        if Op::ALL.iter().any(|(s, _)| value.starts_with(s)) {
            return Err(format!("`{text}` has two operators"));
        }
        Ok(Comparison {
            name: name.to_string(),
            op,
            value: value.to_string(),
        })
    }

    fn holds(&self, actual: &str) -> bool {
        self.op.holds(compare(actual, &self.value))
    }
}

/// The comparisons joined with `&&` and `||`: the condition holds when every comparison of one
/// group holds.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Condition {
    any: Vec<Vec<Comparison>>,
}

impl Condition {
    pub(crate) fn parse(text: &str) -> Result<Self, String> {
        let any = text
            .split("||")
            .map(|group| group.split("&&").map(Comparison::parse).collect())
            .collect::<Result<Vec<Vec<_>>, _>>()?;
        Ok(Condition { any })
    }

    fn names(&self) -> impl Iterator<Item = &str> {
        self.any.iter().flatten().map(|c| c.name.as_str())
    }

    /// Whether the condition holds for `values`, the value of each name.
    fn holds(&self, values: &Values) -> Result<bool, String> {
        for group in &self.any {
            let mut all = true;
            for comparison in group {
                if !comparison.holds(values.get(&comparison.name)?) {
                    all = false;
                }
            }
            if all {
                return Ok(true);
            }
        }
        Ok(false)
    }
}

/// When a hit that meets the condition stops the core.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum HitCondition {
    /// The Nth hit, only.
    Exactly(u64),
    /// The Nth hit and every one after it.
    From(u64),
    /// Every Nth hit.
    Every(u64),
}

impl HitCondition {
    pub(crate) fn parse(text: &str) -> Result<Self, String> {
        let text = text.trim();
        let count = |number: &str| {
            parse_integer(number.trim())
                .and_then(|n| u64::try_from(n).ok())
                .ok_or_else(|| {
                    format!("`{text}` is not a hit count: write N, == N, >= N, > N or % N")
                })
        };
        let condition = if let Some(rest) = text.strip_prefix(">=") {
            HitCondition::From(count(rest)?)
        } else if let Some(rest) = text.strip_prefix('>') {
            HitCondition::From(count(rest)?.saturating_add(1))
        } else if let Some(rest) = text.strip_prefix('%') {
            HitCondition::Every(count(rest)?)
        } else if let Some(rest) = text.strip_prefix("==") {
            HitCondition::Exactly(count(rest)?)
        } else {
            HitCondition::Exactly(count(text)?)
        };
        if matches!(condition, HitCondition::Every(0)) {
            return Err(format!("`{text}`: every 0th hit never comes"));
        }
        Ok(condition)
    }

    /// Whether the `hit`th hit (counted from 1) stops the core.
    fn stops(self, hit: u64) -> bool {
        match self {
            HitCondition::Exactly(n) => hit == n,
            HitCondition::From(n) => hit >= n,
            HitCondition::Every(n) => hit.is_multiple_of(n),
        }
    }
}

/// A piece of a log message.
#[derive(Clone, Debug, PartialEq)]
enum Piece {
    Text(String),
    Name(String),
}

/// Text with `{name}` replaced by the value of `name`; `{{` and `}}` are a brace.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct LogMessage {
    pieces: Vec<Piece>,
}

impl LogMessage {
    pub(crate) fn parse(text: &str) -> Result<Self, String> {
        let mut pieces = Vec::new();
        let mut current = String::new();
        let mut chars = text.chars().peekable();
        while let Some(c) = chars.next() {
            match c {
                '{' if chars.peek() == Some(&'{') => {
                    chars.next();
                    current.push('{');
                }
                '}' if chars.peek() == Some(&'}') => {
                    chars.next();
                    current.push('}');
                }
                '{' => {
                    let mut name = String::new();
                    loop {
                        match chars.next() {
                            Some('}') => break,
                            Some(c) => name.push(c),
                            None => return Err(format!("`{text}` has a {{ without its }}")),
                        }
                    }
                    let name = name.trim();
                    if name.is_empty() || name.contains(char::is_whitespace) {
                        return Err(format!("`{{{name}}}` in `{text}` is not one name"));
                    }
                    if !current.is_empty() {
                        pieces.push(Piece::Text(std::mem::take(&mut current)));
                    }
                    pieces.push(Piece::Name(name.to_string()));
                }
                c => current.push(c),
            }
        }
        if !current.is_empty() {
            pieces.push(Piece::Text(current));
        }
        Ok(LogMessage { pieces })
    }

    fn names(&self) -> impl Iterator<Item = &str> {
        self.pieces.iter().filter_map(|piece| match piece {
            Piece::Name(name) => Some(name.as_str()),
            Piece::Text(_) => None,
        })
    }

    /// The message with each name replaced by its value, or by the reason it has none.
    fn render(&self, values: &Values) -> String {
        self.pieces
            .iter()
            .map(|piece| match piece {
                Piece::Text(text) => text.clone(),
                Piece::Name(name) => match values.get(name) {
                    Ok(value) => value.to_string(),
                    Err(error) => format!("<{error}>"),
                },
            })
            .collect()
    }
}

/// The value of each name of a breakpoint, read at a halt: the value, or why there is none.
#[derive(Debug, Default)]
pub(crate) struct Values {
    values: Vec<(String, Result<String, String>)>,
}

impl Values {
    pub(crate) fn insert(&mut self, name: &str, value: Result<String, String>) {
        self.values.push((name.to_string(), value));
    }

    fn get(&self, name: &str) -> Result<&str, String> {
        match self.values.iter().find(|(n, _)| n == name) {
            Some((_, Ok(value))) => Ok(value),
            Some((_, Err(error))) => Err(format!("cannot read `{name}`: {error}")),
            None => Err(format!("`{name}` was not read")),
        }
    }
}

/// What a halt at a breakpoint does, after its condition and hit count.
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum Verdict {
    /// Tell the client, with a note for the stop description when there is one.
    Stop(Option<String>),
    /// Run again without telling the client; the log line to show when there is one.
    Run(Option<String>),
}

/// The condition, hit count and log message of one breakpoint, and its hits.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct BreakpointCondition {
    condition: Option<Condition>,
    hit_condition: Option<HitCondition>,
    log_message: Option<LogMessage>,
    /// The hits that met the condition, since the breakpoint was set.
    hits: u64,
    /// Whether a name is a local variable or a register at the breakpoint, which needs the
    /// frame of the halt; `false` when all are global variables, read without it. Known after
    /// the first hit; data breakpoints halt anywhere and keep `None`.
    needs_frame: Option<bool>,
}

impl BreakpointCondition {
    /// The breakpoint's condition from the DAP fields; `None` when it has none of them. An
    /// empty field is no field (VS Code sends `""` after a condition is deleted).
    pub(crate) fn new(
        condition: Option<&str>,
        hit_condition: Option<&str>,
        log_message: Option<&str>,
    ) -> Result<Option<Self>, String> {
        fn given(field: Option<&str>) -> Option<&str> {
            field.filter(|text| !text.trim().is_empty())
        }
        let condition = given(condition).map(Condition::parse).transpose()?;
        let hit_condition = given(hit_condition).map(HitCondition::parse).transpose()?;
        let log_message = log_message
            .filter(|text| !text.is_empty())
            .map(LogMessage::parse)
            .transpose()?;
        if condition.is_none() && hit_condition.is_none() && log_message.is_none() {
            return Ok(None);
        }
        Ok(Some(BreakpointCondition {
            condition,
            hit_condition,
            log_message,
            hits: 0,
            needs_frame: None,
        }))
    }

    /// The same condition counting from 0, for a breakpoint set again (perhaps elsewhere).
    pub(crate) fn restarted(&self) -> Self {
        BreakpointCondition {
            hits: 0,
            needs_frame: None,
            ..self.clone()
        }
    }

    pub(crate) fn needs_frame(&self) -> Option<bool> {
        self.needs_frame
    }

    pub(crate) fn set_needs_frame(&mut self, needs_frame: bool) {
        self.needs_frame = Some(needs_frame);
    }

    /// The names to read before [`Self::check`], each once.
    pub(crate) fn names(&self) -> Vec<String> {
        let mut names: Vec<String> = Vec::new();
        let all = self
            .condition
            .iter()
            .flat_map(|condition| condition.names())
            .chain(self.log_message.iter().flat_map(|message| message.names()));
        for name in all {
            if !names.iter().any(|n| n == name) {
                names.push(name.to_string());
            }
        }
        names
    }

    /// Checks a hit with the values of [`Self::names`], counting it when it meets the
    /// condition. A name that cannot be read stops the core, with the reason.
    pub(crate) fn check(&mut self, values: &Values) -> Verdict {
        if let Some(condition) = &self.condition {
            match condition.holds(values) {
                Ok(true) => {}
                Ok(false) => return Verdict::Run(None),
                Err(error) => {
                    return Verdict::Stop(Some(format!("condition not checked, {error}")));
                }
            }
        }
        self.hits += 1;
        if let Some(hit_condition) = self.hit_condition
            && !hit_condition.stops(self.hits)
        {
            return Verdict::Run(None);
        }
        match &self.log_message {
            Some(message) => Verdict::Run(Some(message.render(values))),
            None => Verdict::Stop(None),
        }
    }
}

/// `value` as an integer: decimal, `0x` hexadecimal, `0b` binary, a character `'a'`, or
/// `true`/`false`. A value as probe-rs shows it may carry more after a space, such as the
/// character of a `char` (`97 'a'`); only the first word is read.
fn parse_integer(value: &str) -> Option<i128> {
    let value = value.trim();
    if let Some(c) = value
        .strip_prefix('\'')
        .and_then(|rest| rest.strip_suffix('\''))
        .and_then(|inner| {
            let mut chars = inner.chars();
            chars.next().filter(|_| chars.next().is_none())
        })
    {
        return Some(c as i128);
    }
    match value {
        "true" => return Some(1),
        "false" => return Some(0),
        _ => {}
    }
    let (negative, digits) = match value.strip_prefix('-') {
        Some(rest) => (true, rest),
        None => (false, value),
    };
    let digits = digits.replace('_', "");
    let magnitude = if let Some(hex) = digits
        .strip_prefix("0x")
        .or_else(|| digits.strip_prefix("0X"))
    {
        i128::from_str_radix(hex, 16).ok()?
    } else if let Some(binary) = digits
        .strip_prefix("0b")
        .or_else(|| digits.strip_prefix("0B"))
    {
        i128::from_str_radix(binary, 2).ok()?
    } else {
        digits.parse::<i128>().ok()?
    };
    Some(if negative { -magnitude } else { magnitude })
}

/// Compares the value read from the target with the value of a condition.
fn compare(actual: &str, expected: &str) -> Ordering {
    let actual = actual.trim();
    let expected = expected.trim();
    // probe-rs shows some values with more after the number, such as `97 'a'`.
    let first_word = actual.split_whitespace().next().unwrap_or(actual);
    if let (Some(a), Some(e)) = (
        parse_integer(actual).or_else(|| parse_integer(first_word)),
        parse_integer(expected),
    ) {
        return a.cmp(&e);
    }
    if let (Ok(a), Ok(e)) = (first_word.parse::<f64>(), expected.parse::<f64>())
        && let Some(ordering) = a.partial_cmp(&e)
    {
        return ordering;
    }
    let unquote = |text: &str| -> String {
        text.strip_prefix('"')
            .and_then(|rest| rest.strip_suffix('"'))
            .unwrap_or(text)
            .to_string()
    };
    unquote(actual).cmp(&unquote(expected))
}

/// Whether a value from the server's `evaluate` is a value, rather than the reason there is
/// none (`<invalid expression "nosuch">`, `<unavailable: ...>` and such).
pub(crate) fn evaluated(result: String) -> Result<String, String> {
    let trimmed = result.trim();
    if trimmed.starts_with('<') && trimmed.ends_with('>') {
        Err(trimmed
            .trim_start_matches('<')
            .trim_end_matches('>')
            .to_string())
    } else {
        Ok(result)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn values(pairs: &[(&str, &str)]) -> Values {
        let mut values = Values::default();
        for (name, value) in pairs {
            values.insert(name, Ok(value.to_string()));
        }
        values
    }

    fn holds(condition: &str, pairs: &[(&str, &str)]) -> bool {
        Condition::parse(condition)
            .unwrap()
            .holds(&values(pairs))
            .unwrap()
    }

    #[test]
    fn comparisons() {
        assert!(holds("v == 3", &[("v", "3")]));
        assert!(!holds("v == 3", &[("v", "2")]));
        assert!(holds("v != 3", &[("v", "2")]));
        assert!(holds("v < 3", &[("v", "2")]));
        assert!(holds("v <= 3", &[("v", "3")]));
        assert!(holds("v > 1", &[("v", "2")]));
        assert!(holds("v >= 2", &[("v", "2")]));
        assert!(!holds("v >= 2", &[("v", "1")]));
        assert!(holds("v==3", &[("v", "3")]));
        assert!(holds("p->state >= 2", &[("p->state", "2")]));
        assert!(!holds("p->state>2", &[("p->state", "2")]));
    }

    #[test]
    fn values_are_integers_when_both_sides_are() {
        assert!(holds("d15 == 0x2a", &[("d15", "42")]));
        assert!(holds("d15 == 42", &[("d15", "0x0000002A")]));
        assert!(holds("c == 'a'", &[("c", "97")]));
        assert!(holds("c == 'a'", &[("c", "97 'a'")]));
        assert!(holds("flag == true", &[("flag", "true")]));
        assert!(holds("flag == 1", &[("flag", "true")]));
        assert!(holds("n < 0", &[("n", "-5")]));
        assert!(holds("n > -6", &[("n", "-5")]));
        assert!(holds("x > 1.5", &[("x", "2.25")]));
        assert!(holds("mode == Running", &[("mode", "Running")]));
        assert!(!holds("mode == Running", &[("mode", "Idle")]));
        assert!(holds("name == \"abc\"", &[("name", "\"abc\"")]));
    }

    #[test]
    fn and_binds_before_or() {
        let both = "v > 1 && k == 3";
        assert!(holds(both, &[("v", "2"), ("k", "3")]));
        assert!(!holds(both, &[("v", "2"), ("k", "2")]));
        assert!(!holds(both, &[("v", "1"), ("k", "3")]));
        let either = "v == 0 || v == 3 && k == 1";
        assert!(holds(either, &[("v", "0"), ("k", "9")]));
        assert!(holds(either, &[("v", "3"), ("k", "1")]));
        assert!(!holds(either, &[("v", "3"), ("k", "2")]));
    }

    #[test]
    fn syntax_errors() {
        assert!(Condition::parse("v ==").is_err());
        assert!(Condition::parse("== 3").is_err());
        assert!(Condition::parse("v").is_err());
        assert!(Condition::parse("v + 1 == 3").is_err());
        assert!(Condition::parse("v == == 3").is_err());
        assert!(Condition::parse("v == 3 &&").is_err());
        // A name that is not ASCII is no panic.
        assert!(Condition::parse("변수 == 1").is_ok());
        assert!(Condition::parse("é>1").is_ok());
        assert!(Condition::parse("변수").is_err());
        assert!(HitCondition::parse("").is_err());
        assert!(HitCondition::parse("% 0").is_err());
        assert!(HitCondition::parse("three").is_err());
        assert!(LogMessage::parse("v = {v").is_err());
        assert!(LogMessage::parse("v = {}").is_err());
    }

    #[test]
    fn hit_conditions() {
        let stops = |text: &str| -> Vec<u64> {
            let condition = HitCondition::parse(text).unwrap();
            (1..=12).filter(|&hit| condition.stops(hit)).collect()
        };
        assert_eq!(stops("3"), vec![3]);
        assert_eq!(stops("== 3"), vec![3]);
        assert_eq!(stops(">= 10"), vec![10, 11, 12]);
        assert_eq!(stops("> 10"), vec![11, 12]);
        assert_eq!(stops("% 4"), vec![4, 8, 12]);
        assert_eq!(stops("%4"), vec![4, 8, 12]);
    }

    #[test]
    fn only_hits_that_meet_the_condition_count() {
        let mut bp = BreakpointCondition::new(Some("v == 3"), Some("2"), None)
            .unwrap()
            .unwrap();
        assert_eq!(bp.names(), vec!["v".to_string()]);
        let verdicts: Vec<Verdict> = ["0", "1", "2", "3", "0", "1", "2", "3", "3"]
            .iter()
            .map(|v| bp.check(&values(&[("v", v)])))
            .collect();
        let stops: Vec<usize> = verdicts
            .iter()
            .enumerate()
            .filter(|(_, verdict)| matches!(verdict, Verdict::Stop(_)))
            .map(|(i, _)| i)
            .collect();
        // The second hit with v == 3 is the eighth halt.
        assert_eq!(stops, vec![7]);
        assert_eq!(bp.restarted().hits, 0);
    }

    #[test]
    fn a_name_that_cannot_be_read_stops() {
        let mut bp = BreakpointCondition::new(Some("nosuch == 1"), None, None)
            .unwrap()
            .unwrap();
        let mut values = Values::default();
        values.insert(
            "nosuch",
            evaluated("<invalid expression \"nosuch\">".to_string()),
        );
        match bp.check(&values) {
            Verdict::Stop(Some(note)) => assert!(note.contains("nosuch"), "{note}"),
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn empty_fields_are_no_condition() {
        assert_eq!(BreakpointCondition::new(None, None, None), Ok(None));
        assert_eq!(
            BreakpointCondition::new(Some(""), Some("  "), Some("")),
            Ok(None)
        );
    }

    #[test]
    fn log_messages() {
        let mut bp = BreakpointCondition::new(None, None, Some("v = {v}, k = {k} {{ok}}"))
            .unwrap()
            .unwrap();
        assert_eq!(bp.names(), vec!["v".to_string(), "k".to_string()]);
        let verdict = bp.check(&values(&[("v", "2"), ("k", "3")]));
        assert_eq!(verdict, Verdict::Run(Some("v = 2, k = 3 {ok}".to_string())));

        let mut values = Values::default();
        values.insert("v", Ok("2".to_string()));
        values.insert("k", Err("not a variable".to_string()));
        match bp.check(&values) {
            Verdict::Run(Some(line)) => assert!(line.starts_with("v = 2, k = <"), "{line}"),
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn a_log_point_with_a_condition_logs_only_when_it_holds() {
        let mut bp = BreakpointCondition::new(Some("v == 1"), None, Some("hit {v}"))
            .unwrap()
            .unwrap();
        assert_eq!(bp.check(&values(&[("v", "0")])), Verdict::Run(None));
        assert_eq!(
            bp.check(&values(&[("v", "1")])),
            Verdict::Run(Some("hit 1".to_string()))
        );
    }
}
