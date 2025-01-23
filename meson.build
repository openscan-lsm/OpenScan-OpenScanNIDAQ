project(
    'OpenScanNIDAQ',
    'c',
    default_options: [
        'warning_level=3',
        'werror=true',
    ],
)

cc = meson.get_compiler('c')

if cc.get_id() != 'msvc' and cc.get_id() != 'clang-cl'
    error('Unsupported compiler (requires msvc or clang-cl')
endif

if build_machine.cpu_family() == 'x86_64'
    programsx86 = 'C:/Program Files (x86)'
else
    programsx86 = 'C:/Program Files'
endif

host_cpu = host_machine.cpu_family()

daqmx_dir = (programsx86 /
    'National Instruments/Shared/ExternalCompilerSupport/C')
daqmx_inc = include_directories(daqmx_dir / 'include')
if host_cpu == 'x86_64'
    daqmx_libdir = daqmx_dir / 'lib64/msvc'
else
    daqmx_libdir = daqmx_dir / 'lib32/msvc'
endif

daqmx_dep = declare_dependency(
    dependencies: cc.find_library(
        'NIDAQmx',
        dirs: daqmx_libdir,
        has_headers: 'NIDAQmx.h',
        header_include_directories: daqmx_inc,
    ),
    include_directories: daqmx_inc,
)

openscandevicelib_dep = dependency(
    'OpenScanDeviceLib',
    fallback: ['OpenScanLib', 'OpenScanDeviceLib'],
    static: true,
    default_options: [
        'devicelib=enabled',
        'apilib=disabled',
        'docs=disabled',
        'tests=disabled',
    ],
)

ssstr_dep = dependency(
    'ssstr',
    fallback: 'ssstr',
)

src = files(
    'Acquisition.c',
    'Clock.c',
    'Detector.c',
    'NIDAQ_OpenScanDevice.c',
    'NIDAQ_OpenScanModule.c',
    'NIDAQ_OpenScanSettings.c',
    'OScNIDAQ.c',
    'ParkUnpark.c',
    'Scanner.c',
    'Waveform.c',
)

osdev = shared_module(
    'OpenScanNIDAQ',
    src,
    name_suffix: 'osdev',
    dependencies: [
        daqmx_dep,
        openscandevicelib_dep,
        ssstr_dep,
    ],
)

subdir('DumpWaveform')
