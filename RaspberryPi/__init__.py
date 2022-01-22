#
# Copyright 2020, William Glasford
#
#  Mars Rover:
#
#  This is the Raspberry Pi code to run the Mars Rover.  The main loop waits on the UART 
#  for a command from the Raspberry Pi.  Once received, the command is executed and 
#  a response is sent back.
#
# Mods:     02/05/20  Initial Release.
#
# Copyright 2020, William Glasford
#
#  This is the init file that starts things off.
#
# Mods:     02/05/20  Initial Release.

from flask import Flask
from config import Config

def create_app():
    app = Flask(__name__)
    app.config.from_object(Config)

    return app

app = create_app()
    
from app import routes
