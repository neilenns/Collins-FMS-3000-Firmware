import os
Import("env")

# The names for the different FMS positions in the cockpit. Used
# to generate output filename of the compiled firmware.
board_position_name = ["pilot", "co-pilot"]

# Get the version number from the build environment.
firmware_version = os.environ.get('VERSION', "2.2.0")
raw_board_number = os.environ.get('BOARD_NUMBER', "1")

# Convert board_number to origin 0 so it can be used to index into board_position_name.
board_number = int(raw_board_number) - 1

print(
    f'Using version {firmware_version} position {board_position_name[board_number]} ({raw_board_number}) for the build')

# Append the version to the build defines so it gets baked into the firmware
env.Append(CPPDEFINES=[
    f'BUILD_VERSION={firmware_version}',
    f'MOBIFLIGHT_NAME=FMS 3000 - {raw_board_number}'
])

# Set the output filename to the name of the board and the version
env.Replace(
    PROGNAME=f'firmware_{env["PIOENV"]}_{board_position_name[board_number]}_{firmware_version.replace(".", "_")}')
