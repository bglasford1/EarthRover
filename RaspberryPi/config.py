#
# Copyright 2020, William Glasford
#
#  Mars Rover:
#
#  This file contains the configuration variables.
#
# Mods:     02/05/20  Initial Release.

import os

class Config(object):
    SECRET_KEY = os.environ.get('SECRET_KEY') or 'super-secret-key'
