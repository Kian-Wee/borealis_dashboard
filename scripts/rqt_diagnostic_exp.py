#!/usr/bin/env python

import sys

from rqt_diagnostic_exp.dashboard import Dashboard
from rqt_gui.main import Main

plugin = 'rqt_diagnostic_exp'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))