from building import *
Import('rtconfig')

src   = []
cwd   = GetCurrentDir()

# add mlx90392 src files.
if GetDepend('PKG_USING_MLX90397'):
    src += Glob('src/mlx9039x.c')

if GetDepend('PKG_MLX90397_USING_SENSOR_V1'):
    src += Glob('src/sensor_melexis_mlx9039x.c')

if GetDepend('PKG_USING_MLX90397_SAMPLE'):
    src += Glob('examples/mlx9039x_sample.c')

# add mlx90397 include path.
path  = [cwd + '/inc']

# add src and include to group.
group = DefineGroup('mlx90397', src, depend = ['PKG_USING_MLX90397'], CPPPATH = path)

Return('group')
