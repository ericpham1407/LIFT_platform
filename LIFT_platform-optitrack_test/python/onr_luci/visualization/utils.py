''' visualization utilities '''

import numpy as np

try:
    import OpenGL.GL as gl
except ImportError:
    # workaround for Mac OS OpenGL import
    try:
        from ctypes import util
        orig_util_find_library = util.find_library
        def _new_util_find_library( name ):
            res = orig_util_find_library( name )
            if res:
                return res
            return '/System/Library/Frameworks/'+name+'.framework/'+name
        util.find_library = _new_util_find_library
        import OpenGL.GL as gl
    except ImportError as e:
        # at this point OpenGL is probably not available
        raise e

def depth_colormap(v: np.ndarray, v_min: float = None, v_max: float = None):
    ''' RGB colormap for depth data '''
    if v_min is None:
        v_min = v.min()
    if v_max is None:
        v_max = v.max()
    v_rel = (v - v_min) / (v_max - v_min)
    rgb = np.array([
        np.interp(v_rel, [0,0.5,1.0], [0,.8, 1]),
        np.interp(v_rel, [0,0.5,1.0], [0,.3, 1]),
        np.interp(v_rel, [0,0.5,1.0], [.5,.5,0])
    ]).T
    return rgb * 255

class ImageTexture:
    '''
    class to help manage an image texture
    '''
    def __init__(self):
        self._texture = gl.glGenTextures(1)
        gl.glBindTexture(gl.GL_TEXTURE_2D, self._texture)
        gl.glTexParameteri(
            gl.GL_TEXTURE_2D,
            gl.GL_TEXTURE_WRAP_S,
            gl.GL_CLAMP_TO_EDGE)
        gl.glTexParameteri(
            gl.GL_TEXTURE_2D,
            gl.GL_TEXTURE_WRAP_T,
            gl.GL_CLAMP_TO_EDGE)

    @property
    def id(self) -> int:
        ''' get the texture id '''
        return self._texture

    def update_depth(self, depth: np.ndarray, min_depth: float = None, max_depth: float = None):
        ''' update for depth coloring '''

        color = depth_colormap(depth.T, v_min = min_depth, v_max = max_depth).astype(np.uint8)

        gl.glBindTexture(gl.GL_TEXTURE_2D, self._texture)

        gl.glTexImage2D(
            gl.GL_TEXTURE_2D,
            0,
            gl.GL_RGB,
            color.shape[1],
            color.shape[0],
            0,
            gl.GL_RGB,
            gl.GL_UNSIGNED_BYTE,
            color
        )
        gl.glGenerateMipmap(gl.GL_TEXTURE_2D)

    def update_color(self, color: np.ndarray):
        ''' update for rgb coloring '''
        gl.glBindTexture(gl.GL_TEXTURE_2D, self._texture)

        color = color.astype(np.uint8)

        gl.glTexImage2D(
            gl.GL_TEXTURE_2D,
            0,
            gl.GL_RGB,
            color.shape[1],
            color.shape[0],
            0,
            gl.GL_RGB,
            gl.GL_UNSIGNED_BYTE,
            color
        )

        gl.glGenerateMipmap(gl.GL_TEXTURE_2D)
