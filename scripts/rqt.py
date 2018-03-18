#!/usr/bin/env python

import sys

from rqt_gui.main import Main


def add_arguments(parser):
    group = parser.add_argument_group('Options for rqt_multiplot plugin')
    group.add_argument(
        '--multiplot-config',
        default="",
        action='store',
        help='Load an xml plot configuration')
    group.add_argument(
        '--multiplot-run-all',
        action='store_true',
        help='Run all plots on startup')
    # needs more tlc (i.e. doesn't actually work) - see https://github.com/ethz-asl/rqt_multiplot_plugin/pull/10
    # group.add_argument(
    #     '--multiplot-bag',
    #     default="",
    #     action='store',
    #     help='Load a bag into multiplot')

main = Main()
sys.exit(main.main(sys.argv, standalone='MultiplotPlugin', plugin_argument_provider=add_arguments))