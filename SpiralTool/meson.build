spiraltool_src = [
    'SpiralTool.c',
    '../src/Waveform.c',
]

executable(
    'SpiralTool',
    spiraltool_src,
    c_args: ['-D_CRT_SECURE_NO_WARNINGS'],
    dependencies: [openscandevicelib_dep],
)
