dumpwaveform_src = [
    'DumpWaveform.c',
    '../Waveform.c',
]

dumpwaveform_exe = executable(
    'DumpWaveform',
    dumpwaveform_src,
    c_args: [
        '-D_CRT_SECURE_NO_WARNINGS',
    ],
    dependencies: [
        openscandevicelib_dep,
    ],
)
