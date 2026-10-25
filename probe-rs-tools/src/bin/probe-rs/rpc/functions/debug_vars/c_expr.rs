//! The C expressions a live watch reads, as ORTI files write them: paths to global variables
//! (`OsTcb[1].state`, `p->next->state`, `*p`, an index that is itself an expression:
//! `OsTcb[OsCurrentTask].state`), integers (`42`, `0x2A`, `'a'`), `&path`, casts
//! (`(unsigned char)x`, `(TaskType)x`, `*(uint32 *)0x70001000`), the unary operators `- ! ~`,
//! the binary operators of C with C's precedence, and `c ? a : b`.
//!
//! An expression that is a path alone reads as before: the value as the variable shows it.
//! Anything else is an integer: decimal, or hexadecimal when it is an address (`&x`, a pointer
//! cast).

use std::fmt;

/// A C type of a cast: its words and the `*`s after them.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct CType {
    pub(crate) name: String,
    pub(crate) pointers: usize,
}

impl CType {
    /// The size in bytes and whether it is signed, for the integer types C, `<stdint.h>` and
    /// AUTOSAR (`Platform_Types.h`) name. `None` for any other name (a typedef we do not know).
    fn integer(&self) -> Option<(usize, bool)> {
        let words: Vec<&str> = self
            .name
            .split_whitespace()
            .filter(|w| !matches!(*w, "const" | "volatile"))
            .collect();
        let name = words.join(" ");
        Some(match name.as_str() {
            "char" | "signed char" | "sint8" | "int8" | "int8_t" | "s8" => (1, true),
            "unsigned char" | "uint8" | "uint8_t" | "u8" | "boolean" | "bool" | "_Bool" => {
                (1, false)
            }
            "short" | "short int" | "signed short" | "signed short int" | "sint16" | "int16"
            | "int16_t" | "s16" => (2, true),
            "unsigned short" | "unsigned short int" | "uint16" | "uint16_t" | "u16" => (2, false),
            "int" | "signed" | "signed int" | "long" | "long int" | "signed long" | "sint32"
            | "int32" | "int32_t" | "s32" => (4, true),
            "unsigned" | "unsigned int" | "unsigned long" | "unsigned long int" | "uint32"
            | "uint32_t" | "u32" | "uintptr_t" | "size_t" => (4, false),
            "long long" | "long long int" | "signed long long" | "sint64" | "int64" | "int64_t"
            | "s64" => (8, true),
            "unsigned long long" | "unsigned long long int" | "uint64" | "uint64_t" | "u64" => {
                (8, false)
            }
            _ => return None,
        })
    }

    /// Whether the name is a type: a known one, or `struct`/`enum`/`union` something.
    fn known(&self) -> bool {
        self.integer().is_some()
            || self.name == "void"
            || ["struct ", "enum ", "union "]
                .iter()
                .any(|k| self.name.starts_with(k))
    }
}

/// One step of a path after its first name.
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum Step {
    Member(String),
    Arrow(String),
    Index(Box<Expr>),
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) enum Expr {
    Int(i128),
    /// The `*`s in front, the first name and the steps.
    Path {
        derefs: usize,
        root: String,
        steps: Vec<Step>,
    },
    AddressOf(Box<Expr>),
    /// `*` on what is not a path: the target of a pointer cast.
    Deref(Box<Expr>),
    Cast(CType, Box<Expr>),
    Unary(&'static str, Box<Expr>),
    Binary(&'static str, Box<Expr>, Box<Expr>),
    Cond(Box<Expr>, Box<Expr>, Box<Expr>),
}

#[derive(Clone, Debug, PartialEq)]
enum Token {
    Int(i128),
    Name(String),
    Op(&'static str),
}

/// The operators, longest first, so that `<=` is not read as `<`.
const OPERATORS: &[&str] = &[
    "<<", ">>", "->", "<=", ">=", "==", "!=", "&&", "||", "(", ")", "[", "]", ".", "+", "-", "*",
    "/", "%", "<", ">", "&", "|", "^", "!", "~", "?", ":",
];

/// The binary operators, loosest first (C's order).
const LEVELS: &[&[&str]] = &[
    &["||"],
    &["&&"],
    &["|"],
    &["^"],
    &["&"],
    &["==", "!="],
    &["<", "<=", ">", ">="],
    &["<<", ">>"],
    &["+", "-"],
    &["*", "/", "%"],
];

/// `text` as an integer as C writes it: decimal, `0x`, `0b`, octal `0…`, with a `u`/`l`
/// suffix, or a character `'a'`.
fn integer(text: &str) -> Option<i128> {
    let digits = text.trim_end_matches(['u', 'U', 'l', 'L']);
    if let Some(hex) = digits
        .strip_prefix("0x")
        .or_else(|| digits.strip_prefix("0X"))
    {
        i128::from_str_radix(hex, 16).ok()
    } else if let Some(binary) = digits
        .strip_prefix("0b")
        .or_else(|| digits.strip_prefix("0B"))
    {
        i128::from_str_radix(binary, 2).ok()
    } else if digits.len() > 1 && digits.starts_with('0') {
        i128::from_str_radix(&digits[1..], 8).ok()
    } else {
        digits.parse().ok()
    }
}

fn tokens(text: &str) -> Result<Vec<Token>, String> {
    let mut out = Vec::new();
    let mut rest = text;
    while let Some(c) = rest.chars().next() {
        if c.is_whitespace() {
            rest = &rest[c.len_utf8()..];
        } else if c == '\'' {
            let mut chars = rest[1..].chars();
            let (value, len) = match (chars.next(), chars.next(), chars.next()) {
                (Some('\\'), Some(e), Some('\'')) => {
                    let value = match e {
                        'n' => 10,
                        'r' => 13,
                        't' => 9,
                        '0' => 0,
                        other => other as i128,
                    };
                    (value, 4)
                }
                (Some(c), Some('\''), _) => (c as i128, 2 + c.len_utf8()),
                _ => return Err(format!("`{rest}`: a character is written 'a'")),
            };
            out.push(Token::Int(value));
            rest = &rest[len..];
        } else if c.is_ascii_alphanumeric() || c == '_' {
            let end = rest
                .find(|c: char| !(c.is_ascii_alphanumeric() || c == '_'))
                .unwrap_or(rest.len());
            let word = &rest[..end];
            if c.is_ascii_digit() {
                out.push(Token::Int(
                    integer(word).ok_or_else(|| format!("`{word}` is not a number"))?,
                ));
            } else {
                out.push(Token::Name(word.to_string()));
            }
            rest = &rest[end..];
        } else {
            let op = OPERATORS
                .iter()
                .find(|op| rest.starts_with(**op))
                .ok_or_else(|| format!("`{c}` is not read here"))?;
            out.push(Token::Op(op));
            rest = &rest[op.len()..];
        }
    }
    Ok(out)
}

struct Parser {
    tokens: Vec<Token>,
    at: usize,
}

impl Parser {
    fn peek(&self) -> Option<&Token> {
        self.tokens.get(self.at)
    }

    fn peek_op(&self) -> Option<&'static str> {
        match self.peek() {
            Some(Token::Op(op)) => Some(op),
            _ => None,
        }
    }

    fn expect(&mut self, op: &str) -> Result<(), String> {
        if self.peek_op() == Some(op) {
            self.at += 1;
            Ok(())
        } else {
            Err(format!("`{op}` expected"))
        }
    }

    fn conditional(&mut self) -> Result<Expr, String> {
        let condition = self.binary(0)?;
        if self.peek_op() != Some("?") {
            return Ok(condition);
        }
        self.at += 1;
        let then = self.conditional()?;
        self.expect(":")?;
        let otherwise = self.conditional()?;
        Ok(Expr::Cond(
            Box::new(condition),
            Box::new(then),
            Box::new(otherwise),
        ))
    }

    fn binary(&mut self, level: usize) -> Result<Expr, String> {
        let Some(ops) = LEVELS.get(level) else {
            return self.unary();
        };
        let mut left = self.binary(level + 1)?;
        while let Some(op) = self.peek_op().filter(|op| ops.contains(op)) {
            self.at += 1;
            let right = self.binary(level + 1)?;
            left = Expr::Binary(op, Box::new(left), Box::new(right));
        }
        Ok(left)
    }

    /// A cast at the `(` here: its type, and the index after the `)`.
    fn cast_at(&self) -> Option<(CType, usize)> {
        if self.peek_op() != Some("(") {
            return None;
        }
        let mut at = self.at + 1;
        let mut words = Vec::new();
        while let Some(Token::Name(word)) = self.tokens.get(at) {
            words.push(word.clone());
            at += 1;
        }
        if words.is_empty() {
            return None;
        }
        let mut pointers = 0;
        while self.tokens.get(at) == Some(&Token::Op("*")) {
            pointers += 1;
            at += 1;
        }
        if self.tokens.get(at) != Some(&Token::Op(")")) {
            return None;
        }
        let ty = CType {
            name: words.join(" "),
            pointers,
        };
        // `(a) - b` is a subtraction unless `a` is a type we know; `(TaskType)x` is a cast.
        let next = self.tokens.get(at + 1);
        let operand = matches!(
            next,
            Some(Token::Name(_) | Token::Int(_) | Token::Op("(" | "!" | "~"))
        );
        let ambiguous = matches!(next, Some(Token::Op("-" | "+" | "*" | "&")));
        let cast = words.len() > 1 || pointers > 0 || ty.known() || (operand && !ambiguous);
        (cast && next.is_some()).then_some((ty, at + 1))
    }

    fn unary(&mut self) -> Result<Expr, String> {
        if let Some((ty, after)) = self.cast_at() {
            self.at = after;
            return Ok(Expr::Cast(ty, Box::new(self.unary()?)));
        }
        match self.peek_op() {
            Some(op @ ("-" | "!" | "~" | "+")) => {
                self.at += 1;
                let inner = self.unary()?;
                Ok(if op == "+" {
                    inner
                } else {
                    Expr::Unary(op, Box::new(inner))
                })
            }
            Some("*") => {
                self.at += 1;
                Ok(match self.unary()? {
                    Expr::Path {
                        derefs,
                        root,
                        steps,
                    } => Expr::Path {
                        derefs: derefs + 1,
                        root,
                        steps,
                    },
                    other => Expr::Deref(Box::new(other)),
                })
            }
            Some("&") => {
                self.at += 1;
                match self.unary()? {
                    path @ Expr::Path { .. } => Ok(Expr::AddressOf(Box::new(path))),
                    _ => Err("`&` takes a variable".into()),
                }
            }
            _ => self.postfix(),
        }
    }

    fn postfix(&mut self) -> Result<Expr, String> {
        let root = match self.peek().cloned() {
            Some(Token::Int(n)) => {
                self.at += 1;
                return Ok(Expr::Int(n));
            }
            Some(Token::Op("(")) => {
                self.at += 1;
                let inner = self.conditional()?;
                self.expect(")")?;
                if matches!(self.peek_op(), Some("." | "->" | "[")) {
                    return Err(
                        "a member or element of a parenthesized expression is not read".into(),
                    );
                }
                return Ok(inner);
            }
            Some(Token::Name(name)) => {
                self.at += 1;
                name
            }
            Some(Token::Op(op)) => return Err(format!("`{op}` needs a value before it")),
            None => return Err("a value is missing at the end".into()),
        };
        if self.peek_op() == Some("(") {
            return Err(format!("`{root}(…)`: function calls are not read"));
        }
        let mut steps = Vec::new();
        loop {
            match self.peek_op() {
                Some(op @ ("." | "->")) => {
                    self.at += 1;
                    let Some(Token::Name(member)) = self.peek().cloned() else {
                        return Err(format!("a member name after `{op}`"));
                    };
                    self.at += 1;
                    steps.push(if op == "." {
                        Step::Member(member)
                    } else {
                        Step::Arrow(member)
                    });
                }
                Some("[") => {
                    self.at += 1;
                    let index = self.conditional()?;
                    self.expect("]")?;
                    steps.push(Step::Index(Box::new(index)));
                }
                _ => break,
            }
        }
        Ok(Expr::Path {
            derefs: 0,
            root,
            steps,
        })
    }
}

/// Reads an expression.
pub(crate) fn parse(text: &str) -> Result<Expr, String> {
    let mut parser = Parser {
        tokens: tokens(text)?,
        at: 0,
    };
    if parser.tokens.is_empty() {
        return Err("an expression is missing".into());
    }
    let expr = parser.conditional()?;
    match parser.peek() {
        None => Ok(expr),
        Some(_) => Err(format!("`{text}`: an operator is missing")),
    }
}

/// What a path reads as.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct Leaf {
    /// The value as the variable shows it.
    pub(crate) shown: String,
    /// The integer it holds (a pointer: the address it holds), if it is one.
    pub(crate) value: Option<i128>,
    /// Where it is, if in memory.
    pub(crate) address: Option<u64>,
}

/// The target the expression is read on.
pub(crate) trait Target {
    /// The variable at a path written as the server's paths are (`OsTcb[3].state`, `*p`).
    fn path(&mut self, path: &str) -> Result<Leaf, String>;
    /// `size` bytes at `address`, little endian.
    fn read(&mut self, address: u64, size: usize) -> Result<u64, String>;
}

/// A value and what it is: an address (with the type pointed to) or an integer.
#[derive(Clone, Debug, PartialEq, Eq)]
struct Value {
    n: i128,
    /// The pointed-to type, for an address (`&x`: none known; a pointer cast: its type).
    pointer: Option<Option<CType>>,
}

impl Value {
    fn int(n: i128) -> Self {
        Self { n, pointer: None }
    }
}

impl fmt::Display for Value {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.pointer.is_some() {
            write!(f, "{:#010x}", self.n)
        } else {
            write!(f, "{}", self.n)
        }
    }
}

/// The text of a path with its indexes read: `OsTcb[3].state`.
fn path_text(
    derefs: usize,
    root: &str,
    steps: &[Step],
    target: &mut dyn Target,
) -> Result<String, String> {
    let mut text = "*".repeat(derefs) + root;
    for step in steps {
        match step {
            Step::Member(member) => text = format!("{text}.{member}"),
            Step::Arrow(member) => text = format!("{text}->{member}"),
            Step::Index(index) => {
                let index = eval(index, target)?.n;
                if index < 0 {
                    return Err(format!("`{text}[{index}]`: a negative index"));
                }
                text = format!("{text}[{index}]");
            }
        }
    }
    Ok(text)
}

/// Truncates `n` to `size` bytes, sign-extended when `signed`.
fn fit(n: i128, size: usize, signed: bool) -> i128 {
    let bits = 8 * size as u32;
    let mask = (1i128 << bits) - 1;
    let n = n & mask;
    if signed && n >> (bits - 1) & 1 == 1 {
        n - (1i128 << bits)
    } else {
        n
    }
}

fn eval(expr: &Expr, target: &mut dyn Target) -> Result<Value, String> {
    Ok(match expr {
        Expr::Int(n) => Value::int(*n),
        Expr::Path {
            derefs,
            root,
            steps,
        } => {
            let path = path_text(*derefs, root, steps, target)?;
            let leaf = target.path(&path)?;
            let n = leaf
                .value
                .ok_or_else(|| format!("`{path}` = {} is not an integer", leaf.shown))?;
            Value::int(n)
        }
        Expr::AddressOf(path) => {
            let Expr::Path {
                derefs,
                root,
                steps,
            } = path.as_ref()
            else {
                return Err("`&` takes a variable".into());
            };
            let path = path_text(*derefs, root, steps, target)?;
            let address = target
                .path(&path)?
                .address
                .ok_or_else(|| format!("`{path}` is not in memory"))?;
            Value {
                n: address as i128,
                pointer: Some(None),
            }
        }
        Expr::Deref(inner) => {
            let value = eval(inner, target)?;
            let Some(Some(ty)) = value.pointer else {
                return Err(
                    "`*` takes a pointer: cast it first, as in *(uint32 *)0x70001000".into(),
                );
            };
            let (size, signed) = if ty.pointers > 0 {
                (4, false)
            } else {
                ty.integer().ok_or_else(|| {
                    format!("`*({} *)`: the size of `{}` is not known", ty.name, ty.name)
                })?
            };
            let raw = target.read(value.n as u64, size)?;
            let n = fit(raw as i128, size, signed);
            if ty.pointers > 0 {
                Value {
                    n,
                    pointer: Some(Some(CType {
                        name: ty.name.clone(),
                        pointers: ty.pointers - 1,
                    })),
                }
            } else {
                Value::int(n)
            }
        }
        Expr::Cast(ty, inner) => {
            let value = eval(inner, target)?;
            if ty.pointers > 0 {
                Value {
                    n: fit(value.n, 4, false),
                    pointer: Some(Some(CType {
                        name: ty.name.clone(),
                        pointers: ty.pointers - 1,
                    })),
                }
            } else if let Some((size, signed)) = ty.integer() {
                Value::int(fit(value.n, size, signed))
            } else {
                // A typedef we do not know: the value as it is.
                Value::int(value.n)
            }
        }
        Expr::Unary(op, inner) => {
            let n = eval(inner, target)?.n;
            Value::int(match *op {
                "-" => n.checked_neg().ok_or("the value is too large")?,
                "!" => i128::from(n == 0),
                _ => !n,
            })
        }
        Expr::Binary(op @ ("&&" | "||"), left, right) => {
            let left = eval(left, target)?.n != 0;
            if left == (*op == "||") {
                return Ok(Value::int(i128::from(left)));
            }
            Value::int(i128::from(eval(right, target)?.n != 0))
        }
        Expr::Binary(op, left, right) => {
            let a = eval(left, target)?;
            let b = eval(right, target)?;
            let pointer = match *op {
                "+" | "-" => a.pointer.clone().filter(|_| b.pointer.is_none()),
                _ => None,
            };
            let (x, y) = (a.n, b.n);
            let n = match *op {
                "+" => x.checked_add(y),
                "-" => x.checked_sub(y),
                "*" => x.checked_mul(y),
                "/" | "%" if y == 0 => return Err(format!("`{op}` by 0")),
                "/" => x.checked_div(y),
                "%" => x.checked_rem(y),
                "&" => Some(x & y),
                "|" => Some(x | y),
                "^" => Some(x ^ y),
                "<<" | ">>" => {
                    let shift = u32::try_from(y)
                        .ok()
                        .filter(|s| *s < 64)
                        .ok_or_else(|| format!("`{op} {y}`: shift by 0 to 63"))?;
                    if *op == "<<" {
                        x.checked_shl(shift)
                    } else {
                        x.checked_shr(shift)
                    }
                }
                "==" => Some(i128::from(x == y)),
                "!=" => Some(i128::from(x != y)),
                "<" => Some(i128::from(x < y)),
                "<=" => Some(i128::from(x <= y)),
                ">" => Some(i128::from(x > y)),
                ">=" => Some(i128::from(x >= y)),
                _ => return Err(format!("`{op}` is not read")),
            }
            .ok_or("the value is too large")?;
            Value { n, pointer }
        }
        Expr::Cond(condition, then, otherwise) => {
            if eval(condition, target)?.n != 0 {
                eval(then, target)?
            } else {
                eval(otherwise, target)?
            }
        }
    })
}

/// What a live watch shows for `text`: a path alone as the variable shows it, anything else as
/// an integer.
pub(crate) fn evaluate(text: &str, target: &mut dyn Target) -> Result<String, String> {
    let expr = parse(text).map_err(|error| format!("not supported: {error}"))?;
    if let Expr::Path {
        derefs,
        root,
        steps,
    } = &expr
    {
        let path = path_text(*derefs, root, steps, target)?;
        return Ok(target.path(&path)?.shown);
    }
    Ok(eval(&expr, target)?.to_string())
}

/// The integer a variable's value shows, if it shows one: `42`, `-3`, `0x2A`, `97 'a'`, `true`.
pub(crate) fn shown_integer(shown: &str) -> Option<i128> {
    let shown = shown.trim();
    match shown {
        "true" => return Some(1),
        "false" => return Some(0),
        _ => {}
    }
    let word = shown.split_whitespace().next()?;
    let (negative, digits) = match word.strip_prefix('-') {
        Some(rest) => (true, rest),
        None => (false, word),
    };
    let n = match digits
        .strip_prefix("0x")
        .or_else(|| digits.strip_prefix("0X"))
    {
        Some(hex) => i128::from_str_radix(hex, 16).ok()?,
        None => digits.parse::<i128>().ok()?,
    };
    Some(if negative { -n } else { n })
}

#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use super::*;

    /// Variables by path, and memory.
    struct Fake {
        paths: HashMap<&'static str, Leaf>,
        memory: HashMap<u64, u64>,
        asked: Vec<String>,
    }

    impl Target for Fake {
        fn path(&mut self, path: &str) -> Result<Leaf, String> {
            self.asked.push(path.to_string());
            self.paths
                .get(path)
                .cloned()
                .ok_or_else(|| format!("`{path}` is not a global variable"))
        }

        fn read(&mut self, address: u64, size: usize) -> Result<u64, String> {
            let word = self.memory.get(&address).ok_or("not readable")?;
            Ok(word & ((1u128 << (8 * size)) - 1) as u64)
        }
    }

    fn leaf(shown: &str, value: Option<i128>, address: Option<u64>) -> Leaf {
        Leaf {
            shown: shown.into(),
            value,
            address,
        }
    }

    fn fake() -> Fake {
        Fake {
            paths: HashMap::from([
                ("OsCurrentTask", leaf("2", Some(2), Some(0x7000_0010))),
                ("OsTcb[2].state", leaf("3", Some(3), Some(0x7000_0048))),
                (
                    "OsTcb[0]",
                    leaf("OsTcb @ 0x70000020", None, Some(0x7000_0020)),
                ),
                (
                    "OsTcb[1]",
                    leaf("OsTcb @ 0x70000034", None, Some(0x7000_0034)),
                ),
                (
                    "OsRunningTcb",
                    leaf("*OsTcb @ 0x70000034", Some(0x7000_0034), Some(0x7000_0014)),
                ),
                ("OsRunningTcb->prio", leaf("2", Some(2), None)),
                ("g_signed", leaf("-1", Some(-1), Some(0x7000_0018))),
                ("g_name", leaf("\"abc\"", None, Some(0x7000_0060))),
            ]),
            memory: HashMap::from([(0x7000_0018, 0xFFFF_FFFF), (0x7000_0010, 0x1_0002)]),
            asked: Vec::new(),
        }
    }

    fn read(text: &str) -> Result<String, String> {
        evaluate(text, &mut fake())
    }

    #[test]
    fn a_path_alone_shows_as_the_variable() {
        assert_eq!(read("OsTcb[2].state").unwrap(), "3");
        assert_eq!(read("OsTcb[0]").unwrap(), "OsTcb @ 0x70000020");
        assert_eq!(read("OsRunningTcb").unwrap(), "*OsTcb @ 0x70000034");
        assert_eq!(read("OsRunningTcb -> prio").unwrap(), "2");
    }

    #[test]
    fn an_index_read_from_the_target() {
        let mut target = fake();
        assert_eq!(
            evaluate("OsTcb[OsCurrentTask].state", &mut target).unwrap(),
            "3"
        );
        assert_eq!(target.asked, ["OsCurrentTask", "OsTcb[2].state"]);
        assert_eq!(read("OsTcb[OsCurrentTask - 1 + 1].state").unwrap(), "3");
    }

    #[test]
    fn addresses_and_pointers() {
        assert_eq!(read("&OsTcb[1]").unwrap(), "0x70000034");
        assert_eq!(read("OsRunningTcb == &OsTcb[1]").unwrap(), "1");
        assert_eq!(read("(uint32)&OsTcb[0]").unwrap(), "1879048224");
        assert_eq!(read("&OsTcb[0] + 4").unwrap(), "0x70000024");
        assert_eq!(read("*(uint16 *)0x70000010").unwrap(), "2");
        assert_eq!(read("*(sint8 *)&g_signed").unwrap(), "-1");
        assert_eq!(read("*(unsigned int*)(0x70000018)").unwrap(), "4294967295");
        assert!(read("*0x70000010").unwrap_err().contains("cast it first"));
        assert!(
            read("*(TaskType *)0x70000010")
                .unwrap_err()
                .contains("not known")
        );
    }

    #[test]
    fn casts_and_arithmetic() {
        assert_eq!(read("(unsigned char)g_signed").unwrap(), "255");
        assert_eq!(read("(TaskType)OsCurrentTask").unwrap(), "2");
        assert_eq!(read("(OsCurrentTask) - 1").unwrap(), "1");
        assert_eq!(read("(int)OsCurrentTask - 1").unwrap(), "1");
        assert_eq!(read("OsTcb[2].state & 0x1 | 4 << 1").unwrap(), "9");
        assert_eq!(read("OsCurrentTask == 2 ? 10 : 20").unwrap(), "10");
        assert_eq!(
            read("OsCurrentTask > 5 ? 10 : OsCurrentTask < 3 ? 1 : 2").unwrap(),
            "1"
        );
        assert_eq!(read("OsCurrentTask || nothing").unwrap(), "1");
        assert_eq!(read("!OsCurrentTask && nothing").unwrap(), "0");
        assert_eq!(read("'A' + 1").unwrap(), "66");
        assert_eq!(read("010 + 0b11 + 2u").unwrap(), "13");
    }

    #[test]
    fn what_is_not_read() {
        for (text, why) in [
            ("f(1)", "function calls"),
            ("a = 1", "is not read"),
            ("g_name + 1", "not an integer"),
            ("&(a + 1)", "takes a variable"),
            ("(a).b", "parenthesized"),
            ("1 +", "missing"),
            ("x[1", "`]` expected"),
            ("nothing + 1", "not a global variable"),
            ("1 / 0", "by 0"),
        ] {
            let error = read(text).unwrap_err();
            assert!(error.contains(why), "{text}: {error}");
        }
    }

    #[test]
    fn integers_shown() {
        assert_eq!(shown_integer("42"), Some(42));
        assert_eq!(shown_integer("-3"), Some(-3));
        assert_eq!(shown_integer("97 'a'"), Some(97));
        assert_eq!(shown_integer("0x2A"), Some(42));
        assert_eq!(shown_integer("true"), Some(1));
        assert_eq!(shown_integer("Running"), None);
    }
}
