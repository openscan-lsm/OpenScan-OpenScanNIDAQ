# OpenScan device module using NI DAQ

## How to build

- Install Visual Studio 2019 or later with C++ Desktop Development, and
  [Meson](https://github.com/mesonbuild/meson/releases). Make sure `meson` and
  `ninja` are on the `PATH`.
- Install the latest
  [NI-DAQmx](https://www.ni.com/en-us/support/downloads/drivers/download.ni-daqmx.html).
- Build OpenScan-OpenScanNIDAQ. This is best done in the Developer PowerShell
  for VS 2019 (or later), which can be started from the Start Menu (hint: type
  'developer powershell' into the Start Menu to search).

```pwsh
cd path\to\OpenScan-OpenScanNIDAQ
meson setup builddir --buildtype release
meson compile -C builddir
```

This results in the module `OpenScanNIDAQ.osdev` in `builddir`.

## Code of Conduct

[![Contributor Covenant](https://img.shields.io/badge/Contributor%20Covenant-2.0-4baaaa.svg)](https://github.com/openscan-lsm/OpenScan/blob/main/CODE_OF_CONDUCT.md)
