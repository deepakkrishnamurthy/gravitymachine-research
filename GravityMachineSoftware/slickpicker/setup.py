from distutils.core import setup
from distutils.command.install import install as _install
from distutils import log
from stat import ST_ATIME, ST_MTIME, S_IMODE, ST_MODE
import os, glob, sys
#from _APPNAME_lib import APPNAME,APPVERSION,AUTHOR,DESCRIPTION,YEAR,PAGE,EMAIL
# comment this out for now since slickpicker isn't too complex
# and declare them here

APPNAME='slickpicker'
APPVERSION='0.2'
AUTHOR='ShadowKyogre'
EMAIL='shadowkyogre.public@gmail.com'
DESCRIPTION='A small PyQt color picker widget (can be run on its own)'
PAGE='https://github.com/ShadowKyogre/slickpicker'

if sys.version_info < (3,0,0):
	print("Python 3.x is required!",file=sys.stderr)
	exit(1)

setup(
	name = APPNAME,
	version = APPVERSION,
	author = AUTHOR,
	author_email = EMAIL,
	description = DESCRIPTION,
	url = PAGE,
	license = "GPLv3",
	requires=['PyQt5'],
	py_modules = ['slickpicker'],
	scripts=['slickpicker']
)

