#!/usr/bin/env python

import sys

from borealis_dashboard.dashboard import Dashboard
from rqt_gui.main import Main

plugin = 'borealis_dashboard'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))