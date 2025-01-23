#pragma once
#include <math.h>

namespace inet {

class PDC_map {
	public:
        PDC_map() {
		}
		~PDC_map() {
		}

		// Setter
		void lastSeen(double t) {
			_lastSeen = t;
		}
		void coord(Coord coord) {
			_coord = coord;
		}
		void coord(double x, double y, double z = 0.0) {
			this->coord(Coord(x, y, z));
		}
		void velo(double vx, double vy, double vz) {
			this->velo(Coord(vx, vy, vz));
		}
		void velo(Coord v) {
			_velo = v;
		}

		// Getter
		double lastSeen() {
			return _lastSeen;
		}
		Coord coord() {
			return _coord;
		}
		Coord velo() {
			return _velo;
		}

		void Gamma_Future_map(double v){
		    _Gamma_Future_map = v;
		}
		double Gamma_Future_map() {
		    return _Gamma_Future_map;
		}

		void Gamma_Future_c(double v){
            _Gamma_Future_c = v;
        }
        double Gamma_Future_c() {
            return _Gamma_Future_c;
        }

	protected:
		double _lastSeen;
		Coord _coord;
		Coord _velo;

		double _Gamma_Future_map = 0.0;
		double _Gamma_Future_c = 0.0;
};

}	// Namespace inet
