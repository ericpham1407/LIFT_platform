''' dashboard for testing GPS '''
import numpy as np

import glfw

import imgui
from imgui.integrations.glfw import GlfwRenderer

from onr_luci.hardware.ntrip_client import NtripClient, NtripConfig
from onr_luci.hardware.ublox_gps import GPSFix, GPSClient

from onr_luci.visualization.opengl_window import get_font_file, gl

import pdb

class GPSDashboard():
    '''
    class for creating a window using GLFW,
    adding objects to it to draw with OpenGL,
    and drawing a GUI using imgui
    '''
    window = None
    window_open   = True
    window_height = 800
    window_width = 1250
    imgui_width = 450
    imgui_font = None
    impl = None
    window_resized = False
    should_close = False

    gps_fix: GPSFix = None

    fix_accs: np.ndarray

    xy_hist: np.ndarray

    def __init__(self,
            fullscreen = False):
        self._fullscreen = fullscreen
        self.fix_accs = np.zeros(10000, dtype = np.float32)
        self.xy_hist = None

        self._create_imgui()
        self._create_window()
        self._create_imgui_renderer()

    def draw(self,fix: GPSFix = None):
        ''' draw the window, both OpenGL and Imgui '''
        if not self.window:
            return True

        if fix:
            self.gps_fix = fix
            self.fix_accs = np.roll(self.fix_accs, -1)
            self.fix_accs[-1] = fix.h_acc
            if self.xy_hist is None:
                self.xy_hist = np.ones((2,10000),  dtype = np.float64)
                self.xy_hist[0] *= self.gps_fix.lon
                self.xy_hist[1] *= self.gps_fix.lat
            self.xy_hist = np.roll(self.xy_hist, -1)
            self.xy_hist[0,-1] = self.gps_fix.lon
            self.xy_hist[1,-1] = self.gps_fix.lat

        # gather events
        glfw.poll_events()
        self.impl.process_inputs()

        # draw imgui
        self._draw_imgui()

        # draw opengl
        gl.glClear(gl.GL_COLOR_BUFFER_BIT)

        # renger imgui on top of opengl
        imgui.render()
        self.impl.render(imgui.get_draw_data())

        # push render to window
        glfw.swap_buffers(self.window)

        if glfw.window_should_close(self.window):
            self.should_close = True
        elif self.impl.io.keys_down[glfw.KEY_ESCAPE]:
            self.should_close = True

        return self.should_close

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

        glfw.window_hint(glfw.SAMPLES, 4)

        if self._fullscreen:
            video_mode = glfw.get_video_mode(glfw.get_primary_monitor())
            self.window_width = video_mode.size.width
            self.window_height = video_mode.size.height

        self.window = glfw.create_window(self.window_width,
                                         self.window_height,
                                         "Barc3D",
                                         glfw.get_primary_monitor() if self._fullscreen else None,
                                         None)

        if not self.window:
            glfw.terminate()
            return

        glfw.make_context_current(self.window)
        #glfw.set_window_size_callback(self.window, self._on_resize)
        glfw.set_framebuffer_size_callback(self.window, self._on_resize)

        gl.glEnable(gl.GL_BLEND)
        gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
        gl.glEnable(gl.GL_LINE_SMOOTH)
        gl.glEnable(gl.GL_POLYGON_SMOOTH)
        gl.glEnable(gl.GL_MULTISAMPLE)
        gl.glClearColor(1.0, 1.0, 1.0, 1.0)

    def _create_imgui_renderer(self):
        if self.window:
            self.impl = GlfwRenderer(self.window)

            io = imgui.get_io()
            self.imgui_font = io.fonts.add_font_from_file_ttf(
                get_font_file('DejaVuSans.ttf'), 20,
            )
            self.impl.refresh_font_texture()

    def _on_resize(self, window, w, h):
        if self.window and window:
            if w == 0 or h == 0:
                self.window_open = False
                return
            self.window_width = w
            self.window_height = h
            gl.glViewport(0,0,self.window_width,self.window_height)

    def _draw_imgui(self):
        imgui.new_frame()

        imgui.push_font(self.imgui_font)
        trace_plot_size = (self.imgui_width-20, 200)
        h = 355

        imgui.set_next_window_position(self.window_width - self.imgui_width, 0)
        imgui.set_next_window_size(self.imgui_width, h)
        imgui.begin("Info", closable = False)
        imgui.plot_lines('Horizontal Accuracy',
            self.fix_accs , graph_size = trace_plot_size,
            scale_min = 0, scale_max = 1)
        if self.gps_fix is not None:
            imgui.text(f'Speed: {self.gps_fix.g_speed:0.2f}m/s')
            imgui.text(f'Accuracy: {self.gps_fix.h_acc*100:0.1f}cm')
            imgui.text(f'Number of Satellites: {self.gps_fix.num_sv}')
        imgui.end()

        if self.xy_hist is not None:
            imgui.set_next_window_position(0,0)
            imgui.set_next_window_size(800, 800)
            imgui.begin("GPS Trace", closable = False)
            draw_list = imgui.get_window_draw_list()
            pxy = (self.xy_hist - self.xy_hist.min(axis = 1)[:,np.newaxis]) \
                / (self.xy_hist.max(axis = 1) - self.xy_hist.min(axis = 1) + 1e-9)[:, np.newaxis]\
                * 750 + 25
            #pxy = pxy.astype(np.int32)
            draw_list.add_polyline(pxy.T.tolist(),
                imgui.get_color_u32_rgba(1,1,0,1),
                thickness=3)
            imgui.end()

        imgui.pop_font()

def _main():
    window = GPSDashboard()
    gps = GPSClient()
    ntrip = NtripClient(NtripConfig())
    ntrip.start()

    while not window.should_close:
        data = gps.read()
        if data:
            gps_fix = data[-1]
            # pdb.set_trace()
            print(gps_fix.lat)
        else:
            gps_fix = None

        corrections = ntrip.step(gps_fix)
        for correction in corrections:
            gps.write_corrections(correction)

        window.draw(gps_fix)

    window.close()
    gps.shutdown()
    ntrip.shutdown()


if __name__ == '__main__':
    _main()
