'''
plotting / rendering based on OpenGL and Imgui
'''
import platform
import time

import imgui
import glfw
from imgui.integrations.glfw import GlfwRenderer

from onr_luci.utils.load_utils import get_assets_file
from onr_luci.visualization.utils import gl


def get_font_file(name = 'DejaVuSans.ttf'):
    ''' helper function to get a path to a provided font '''
    filename = get_assets_file(name)
    return filename

class Window():
    '''
    class for creating a window using GLFW,
    adding objects to it to draw with OpenGL,
    and drawing a GUI using imgui

    Keyboard shortcuts:
      ESC: request a window close (Window.draw() returns false)
      SPACE: toggle showing the GUI
      R: toggle screen recording
    '''
    window = None
    window_open:bool = True
    window_height:int = 1080
    window_width:int = 1920
    show_imgui:bool = True
    _last_show_imgui_change: float = None # debounce toggling show via keyboard (space)
    impl = None
    should_close:bool = False

    def __init__(self,
            fullscreen = False,
            window_title: str = 'ONR Luci - UC Berkeley MPC Lab'
            ):
        self._fullscreen = fullscreen
        self._window_title = window_title

        self._create_imgui()
        self._create_window()
        self._create_imgui_renderer()

    def draw(self) -> bool:
        '''
        draw the window, both OpenGL and Imgui
        returns true while the window should stay open
        '''
        if not self.window:
            return True

        # gather events
        glfw.poll_events()
        if not self.window_open:
            # delay as if there were a refresh delay
            time.sleep(1/60)
            return self.should_close

        self.impl.process_inputs()
        self._process_keys()
        self._process_mice()

        # clear screen
        gl.glClear(gl.GL_COLOR_BUFFER_BIT)

        # draw imgui
        imgui.new_frame()
        if self.show_imgui:
            self._draw_imgui()

        # render imgui on top of opengl
        imgui.render()
        if self.show_imgui:
            self.impl.render(imgui.get_draw_data())

        # push render to window
        glfw.swap_buffers(self.window)

        return not self.should_close

    def close(self):
        ''' close the window '''
        if not self.window:
            return
        self.window = None
        glfw.terminate()

    def _create_imgui(self):
        imgui.create_context()

    def _create_window(self):
        if not glfw.init():
            self.window = None
            return

        glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
        glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
        glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
        if platform.system() == 'Darwin':
            glfw.window_hint(glfw.OPENGL_FORWARD_COMPAT, True)

        glfw.window_hint(glfw.SAMPLES, 4)

        if self._fullscreen:
            video_mode = glfw.get_video_mode(glfw.get_primary_monitor())
            self.window_width = video_mode.size.width
            self.window_height = video_mode.size.height

        self.window = glfw.create_window(self.window_width,
                                         self.window_height,
                                         self._window_title,
                                         glfw.get_primary_monitor() if self._fullscreen else None,
                                         None)

        if not self.window:
            glfw.terminate()
            return

        glfw.make_context_current(self.window)
        glfw.set_framebuffer_size_callback(self.window, self._on_resize)
        glfw.set_window_size_callback(self.window, self._on_resize)

        gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glEnable(gl.GL_LINE_SMOOTH)
        gl.glEnable(gl.GL_POLYGON_SMOOTH)
        gl.glEnable(gl.GL_MULTISAMPLE)
        gl.glEnable(gl.GL_CULL_FACE)
        gl.glClearColor(1.0, 1.0, 1.0, 1.0)

        self.window_width, self.window_height = glfw.get_window_size(self.window)

    def _on_resize(self, window, w, h):
        if self.window and window:
            if w == 0 or h == 0:
                # window has been minimized, but not closed
                self.window_open = False
                return
            self.window_open = True
            self.window_width = w
            self.window_height = h
            gl.glViewport(0,0,self.window_width,self.window_height)

    def _create_imgui_renderer(self):
        if self.window:
            self.impl = GlfwRenderer(self.window)
            io = imgui.get_io()

            self.imgui_font = io.fonts.add_font_from_file_ttf(
                get_font_file('DejaVuSans.ttf'), 18)
            self.big_imgui_font = io.fonts.add_font_from_file_ttf(
                get_font_file('DejaVuSans.ttf'), 32)
            self.impl.refresh_font_texture()

    def _draw_imgui(self):
        imgui.push_font(self.imgui_font)

        imgui.set_next_window_position(self.window_width - 300, 0)
        imgui.set_next_window_size(300,self.window_height)

        expanded = imgui.begin("Vehicle Info", closable = False,
            flags = imgui.WINDOW_NO_MOVE | imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_SCROLLBAR)

        imgui.show_test_window()

        imgui.end()
        imgui.pop_font()

    def _process_keys(self):
        if glfw.window_should_close(self.window):
            self.should_close = True

        if not self.impl.io.want_capture_keyboard:
            if self.impl.io.keys_down[glfw.KEY_ESCAPE]:
                self.should_close = True

    def _process_mice(self):
        if not self.impl.io.want_capture_mouse:
            # check key presses here
            pass

if __name__ == '__main__':
    test_window = Window()
    while test_window.draw():
        pass
    test_window.close()
