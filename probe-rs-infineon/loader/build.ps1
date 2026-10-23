# Builds the PFLASH loaders with the AURIX GCC (tricore-elf-gcc):
#   m25_flash_loader_tc27x.bin   TC27x (TC2xx flash module, -DTC27X)
#   m25_flash_loader_tc3xx.bin   TC3xx (DMU), checked on a TC36x
# The binaries are committed because the probe-rs build has no TriCore toolchain.
# After a rebuild, update the loader checks in src/flash.rs (size and SHA-256).
param(
    [string]$Toolchain = (@(
        'C:\Infineon\AURIX-Configuration-Studio-1.0.24\eclipse\tricore-gcc11\bin',
        'C:\Infineon\AURIX-Studio-1.10.40\tools\Compilers\tricore-gcc11\bin'
    ) | Where-Object { Test-Path "$_\tricore-elf-gcc.exe" } | Select-Object -First 1)
)
$ErrorActionPreference = 'Stop'
if (-not $Toolchain) { throw 'AURIX GCC (tricore-gcc11) not found: pass -Toolchain' }
Push-Location $PSScriptRoot
try {
    foreach ($build in @(
            @{ Name = 'm25_flash_loader_tc27x'; Cpu = 'tc27xx'; Defines = @('-DTC27X') },
            @{ Name = 'm25_flash_loader_tc3xx'; Cpu = 'tc38xx'; Defines = @() })) {
        $n = $build.Name
        & "$Toolchain\tricore-elf-gcc.exe" -c -O2 -nostdlib -ffreestanding "-mcpu=$($build.Cpu)" @($build.Defines) -o "$n.o" m25_flash_loader.c
        if ($LASTEXITCODE) { throw "compile failed" }
        & "$Toolchain\tricore-elf-gcc.exe" -nostdlib "-mcpu=$($build.Cpu)" -T m25_flash_loader.ld -o "$n.elf" "$n.o"
        if ($LASTEXITCODE) { throw "link failed" }
        & "$Toolchain\tricore-elf-objcopy.exe" -O binary "$n.elf" "$n.bin"
        if ($LASTEXITCODE) { throw "objcopy failed" }
        Remove-Item "$n.o", "$n.elf"
        $hash = (Get-FileHash "$n.bin" -Algorithm SHA256).Hash
        "{0}.bin {1} bytes SHA-256 {2}" -f $n, (Get-Item "$n.bin").Length, $hash
    }
}
finally {
    Pop-Location
}
