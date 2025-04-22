dumpwaveform_src = [
    'DumpWaveform.c',
    '../src/Waveform.c',
]

executable(
    'DumpWaveform',
    dumpwaveform_src,
    c_args: [
        '-D_CRT_SECURE_NO_WARNINGS',
    ],
    dependencies: [
        openscandevicelib_dep,
    ],
)
