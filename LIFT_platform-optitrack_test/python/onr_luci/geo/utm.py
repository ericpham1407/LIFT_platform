from dataclasses import dataclass, field
import numpy as np

# equations from "Map Projections - A Working Manual", USGS Professional Paper 1395, pg. 60: "Formulas for the Ellipsoid".

# UTM Central Scale Factor
k0 = 0.9996
#WGS84 Ellipsoid
a = 6378137.0
inv_f = 298.257223563
f = 1/inv_f
e2    = 2*f - f**2      # squared eccentricity

e4 = e2**2
e6 = e2**3
er2 = e2 / (1-e2)

_e1 = (1-np.sqrt(1-e2)) / (1 + np.sqrt(1-e2))
_e2 = _e1 ** 2
_e3 = _e1 ** 3
_e4 = _e1 ** 4
_e5 = _e1 ** 5

M1 = 1 - e2 / 4 - 3 * e4  / 64 - 5 * e6 / 256
M2 = 3 * e2 / 8 + 3 * e4 / 32 + 45 * e6 / 1024
M3 = 15 * e4 / 256 + 45 * e6 / 1024
M4 = 35 * e6 / 3072

P2 = (3 / 2 * _e1 - 27 / 32 * _e3 + 269 / 512 * _e5)
P3 = (21 / 16 * _e2 - 55 / 32 * _e4)
P4 = (151 / 96 * _e3 - 417 / 128 * _e5)
P5 = (1097 / 512 * _e4)

@dataclass
class UTMOrigin:
    ''' class to represent a UTM origin and convert to/from wgs84 '''
    lon: float = field(default = 0)
    lat: float = field(default = 0)


    def wgs842utm(self, lon, lat):
        ''' convert lon/lat to x, y, k; k is a scale factor '''
        lam  = np.radians(lon)
        phi  = np.radians(lat)
        lam0 = np.radians(self.lon)
        phi0 = np.radians(self.lat)


        N = a / np.sqrt(1-e2*np.sin(phi)**2)
        T = np.tan(phi)**2
        C = er2 * np.cos(phi)**2
        A = (lam - lam0) * np.cos(phi)
        M = a * (M1 * phi -
                 M2 * np.sin(2*phi) +
                 M3 * np.sin(4*phi) -
                 M4 * np.sin(6*phi))
        M0 = a * (M1 * phi0 -
                 M2 * np.sin(2*phi0) +
                 M3 * np.sin(4*phi0) -
                 M4 * np.sin(6*phi0))


        x = k0 * N * (A +
                      (1-T+C) * A**3 / 6 +
                      (5 - 18 * T + T**2 + 72*C - 58*er2) * A**5 / 120)

        y = k0 * (M - M0 + N * np.tan(phi) * ( A**2 / 2 + (5-T + 9*C + 4*C**2) * A**4 / 24 + (61-58*T + T**2 + 600*C - 330 * er2) * A**6 / 720))
        k = k0 * (1 + (1+C)*A**2/2 + (5-4*T+42*C+13*C**2-28*er2)*A**4/24 + (61-148*T + 16*T**2)*A**6/720)

        return x, y , k

    def utm2wgs84(self, x, y):
        ''' convert x,y to lon, lat'''
        lam0 = np.radians(self.lon)
        phi0 = np.radians(self.lat)

        M0 = a * (M1 * phi0 -
                 M2 * np.sin(2*phi0) +
                 M3 * np.sin(4*phi0) -
                 M4 * np.sin(6*phi0))

        M = y / k0 + M0
        mu = M / a / M1

        phi1 = mu + (P2 * np.sin(2*mu) + \
                     P3 * np.sin(4*mu) + \
                     P4 * np.sin(6*mu) + \
                     P5 * np.sin(8*mu))

        C1 = er2 * np.cos(phi1) **2
        T1 = np.tan(phi1) **2
        N1 = a / np.sqrt(1-e2*np.sin(phi1)**2)
        R1 = a * (1-e2) / (1-e2*np.sin(phi1)**2)**1.5
        D = x / N1 / k0

        phi = phi1 - N1 * np.tan(phi1) / R1 * (D**2 / 2 -
                                               (5 + 3*T1 + 10*C1 - 4*C1**2 - 9*er2) * D**4 / 24 +
                                               (61 + 90 * T1 + 298*C1 + 45*T1**2 - 252 * er2 - 3 * C1**2) * D**6 / 720)

        lam = lam0 + 1/np.cos(phi1) * (D -
                                       (1+2*T1 + C1)*D**3 / 6 +
                                       (5 - 2*C1 + 28*T1 - 3 * C1**2 + 8 * er2 + 24*T1**2) * D**5 / 120)

        lon = np.degrees(lam)
        lat = np.degrees(phi)
        return lon, lat


if __name__ == '__main__':
    origin = UTMOrigin(lon = -75, lat = 40.5)
    print(origin.utm2wgs84(x = 127103.087, y = 1080.67))
    print(origin.wgs842utm(lon = -73.5, lat = 40.5))
