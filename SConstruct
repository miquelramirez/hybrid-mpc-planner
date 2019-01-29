# This SConstruct file is automatically moved to the directory where the generated
# instance-specific problem data resides during the process of generating the executable solver.

import os

HOME = os.path.expanduser("~")

# read variables from the cache, a user's custom.py file or command line arguments
vars = Variables(['variables.cache', 'custom.py'], ARGUMENTS)
vars.Add(BoolVariable('debug', 'Whether this is a debug build', 'no'))
vars.Add(BoolVariable('edebug', 'Extreme debug', 'no'))
vars.Add(EnumVariable('default_compiler', 'Preferred compiler', 'clang++', allowed_values=('g++', 'clang++')))
vars.Add(PathVariable('fs', 'Path where the FS+ library is installed', os.getenv('FS_PATH', ''), PathVariable.PathIsDir))

env = Environment(variables=vars, ENV=os.environ)
env['CXX'] = os.environ.get('CXX', env['default_compiler'])

env.Append( LINKFLAGS = ['-lstdc++' ] )  # Seems to be necessary to avoid compatibility issues at linking time...

if env['edebug']:
	env.Append( CCFLAGS = ['-g', '-DDEBUG', '-DEDEBUG' ] )
	fs_libname = 'fs-edebug'
	lib_name = 'fs_planner_edebug.so'
elif env['debug']:
	env.Append( CCFLAGS = ['-g', '-DDEBUG' ] )
	fs_libname = 'fs-debug'
	lib_name = 'fs_planner_debug.so'
else:
	env.Append( CCFLAGS = ['-O3', '-DNDEBUG' ] )
	fs_libname = 'fs'
	lib_name = 'fs_planner.so'


env.ParseConfig( 'PKG_CONFIG_PATH="{}" pkg-config --cflags --libs {}'.format(env['fs'], fs_libname))

# Header and library directories.
# We include pre-specified '~/local/include' and '~/local/lib' directories in case local versions of some libraries (e.g. Boost) are needed
include_paths = ['.']
isystem_paths = [HOME + '/local/include']
lib_paths = [HOME + '/local/lib']

# Boost Python settings
include_paths.append( '/usr/include/python2.7' )

env.Append( CCFLAGS = '-fPIC' )
env.Append( LIBPATH = [ '/usr/lib/python2.7/config' ] )
env.Append( LIBS = [ '-lboost_python', '-lpython2.7', '-ldl' ] )
env['STATIC_AND_SHARED_OBJECTS_ARE_THE_SAME']=1

env.Append(CPPPATH = [ os.path.abspath(p) for p in include_paths ])
env.Append(CCFLAGS = ['-isystem' + os.path.abspath(p) for p in isystem_paths])

src_objs = []
Export('env')
Export('src_objs')

SConscript( 'src/SConscript')
SConscript( 'src/search/drivers/online/SConscript')
SConscript( 'src/search/algorithms/lookahead/SConscript')
SConscript( 'src/utils/SConscript')


env.SharedLibrary(lib_name, src_objs )
