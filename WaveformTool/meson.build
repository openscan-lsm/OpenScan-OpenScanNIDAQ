waveformtool_src = [
    'WaveformTool.c',
    '../src/Waveform.c',
]

executable(
    'WaveformTool',
    waveformtool_src,
    c_args: [
        '-D_CRT_SECURE_NO_WARNINGS',
    ],
    dependencies: [
        openscandevicelib_dep,
    ],
)
