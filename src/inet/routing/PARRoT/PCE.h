/*
 * PCE.h
 *
 *  Created on: Apr 6, 2020
 *      Author: cedrik
 */

#pragma once

namespace inet {
class PCE {
	public:
		PCE(Ipv4Address destination) {
			destination = destination;
		}
		~PCE() {
		}

		void lastSeen(double v) {
			_lastSeen = v;
		}
		void Q(double v) {
			_qValue = v;
		}
		void V(double v) {
			_rValue = v;
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
		double Q() {
			return _qValue;
		}
		double V() {
			return _rValue;
		}
		unsigned int squNr() {
			return _squNr;
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

        void Vc(double v) {
            _rc = v;
        }
        double Vc() {
            return _rc;
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
		double _qValue;
		double _rValue;
		unsigned int _squNr;

        double _qc;
        double _qmap;
        double _rc;
        double _rmap;

};

} // namespace inet
