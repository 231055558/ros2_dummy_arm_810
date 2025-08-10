import os
import sys

# 直接使用本地的fibre模块
from ..fibre import find_any, find_all

# Standard convention is to add a __version__ attribute to the package
from .version import get_version_str

del get_version_str
