/*
 * PCE.h
 *
 *  Created on: Apr 6, 2020
 *      Author: cedrik
 */

#pragma once

namespace inet {
class PCE_map {
	public:
		PCE_map(Ipv4Address destination) {
			destination = destination;
		}
		~PCE_map() {
		}

		void lastSeen(double v) {
			_lastSeen = v;
		}
		void squNr(unsigned int v) {
			_squNr = v;
		}
		Ipv4Address getDestination() {
			return destination;
		}
		double lastSeen() {
			return _lastSeen;
		}
		unsigned int squNr() {
			return _squNr;
		}

		void Vc(double v) {
            _rc = v;
        }
        double Vc() {
            return _rc;
        }
		void Qc(double v) {
            _qc = v;
        }
		double Qc() {
            return _qc;
        }

		void Qmap(double v){
		    _qmap = v;
		}
		double Qmap() {
		    return _qmap;
		}
        void Vmap(double v) {
            _rmap = v;
        }
        double Vmap() {
            return _rmap;
        }

	protected:
		Ipv4Address destination;
		double _lastSeen;
		unsigned int _squNr;

        double _qc;
        double _qmap;
        double _rc;
        double _rmap;

};

} // namespace inet
