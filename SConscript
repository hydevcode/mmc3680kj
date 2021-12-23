from building import *
Import('rtconfig')

src   = []
cwd   = GetCurrentDir()

src += Glob('sensor_mmc3680kj.c')
src += Glob('lib/mmc3680kj.c')

# add mmc3680kj include path.
path  = [cwd, cwd + '/lib']

# add src and include to group.
group = DefineGroup('mmc3680kj', src, depend = ['PKG_USING_MMC3680KJ'], CPPPATH = path)

Return('group')
