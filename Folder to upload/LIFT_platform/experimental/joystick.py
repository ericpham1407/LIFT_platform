'''
joystick interface via glfw
https://www.glfw.org/docs/3.3/input_guide.html#joystick
'''

import glfw
from OpenGL.GL import *

glfw.init()

window = glfw.create_window(500, 500, 'Joystick Test', None, None)

joy = glfw.JOYSTICK_1

while not glfw.window_should_close(window):
    glfw.poll_events()

    if glfw.joystick_present(joy):
        # analog joysticks
        axes_ptr, axes_count = glfw.get_joystick_axes(joy)
        axes = axes_ptr[:axes_count]
        #print('\t'.join(f'{a:0.2f}' for a in axes))

        # buttons
        btn_ptr, btn_count = glfw.get_joystick_buttons(joy)
        btns = btn_ptr[:btn_count]
        #print(btns)

        # hats, ie. the up/down/left/right rocker
        hat_ptr, hat_count = glfw.get_joystick_hats(joy)
        hats = hat_ptr[:hat_count]
        print(hats)

    glfw.swap_buffers(window)

glfw.terminate()
