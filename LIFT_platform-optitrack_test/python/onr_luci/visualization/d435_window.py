'''
window for viewing livestream camera data
'''

import imgui

from onr_luci.hardware.d435 import D435
from onr_luci.visualization.utils import ImageTexture
from onr_luci.visualization.opengl_window import Window


class CameraWindow(Window):
    ''' d435 visualization window '''
    def __init__(self,
            fullscreen = False,
            window_title: str = 'ONR Luci - UC Berkeley MPC Lab'
            ):
        super().__init__(fullscreen, window_title)
        self.d435 = D435()
        self.rgb_texture = ImageTexture()
        self.dep_texture = ImageTexture()
        self.min_depth = 0
        self.max_depth = 5

    def _draw_imgui(self):

        depth, color, _, _ = self.d435.step()
        if color is not None:
            self.rgb_texture.update_color(color)
        if depth is not None:
            self.dep_texture.update_depth(depth, self.min_depth, self.max_depth)


        imgui.push_font(self.imgui_font)

        imgui.set_next_window_position(self.window_width - 300, 0)
        imgui.set_next_window_size(300,self.window_height)

        expanded = imgui.begin("Camera Feed", closable = False,
            flags = imgui.WINDOW_NO_MOVE | imgui.WINDOW_NO_RESIZE)
        if expanded:
            _, self.min_depth = imgui.slider_float('Min Depth', self.min_depth, -5, 5)
            _, self.max_depth = imgui.slider_float('Max Depth', self.max_depth, 0, 10)
            imgui.separator()
            imgui.text('Color Feed')
            imgui.image(self.rgb_texture.id, 270,270)
            imgui.text('Depth Feed')
            imgui.image(self.dep_texture.id, 270,270)
        imgui.end()


        imgui.pop_font()

if __name__ == '__main__':
    test_window = CameraWindow()
    while test_window.draw():
        pass
    test_window.close()
