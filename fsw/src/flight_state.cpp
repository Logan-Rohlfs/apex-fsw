#include "flight_state.h"

FlightState g_state = {};

const char* phase_name(FlightPhase p) {
    switch (p) {
        case FlightPhase::IDLE:    return "IDLE";
        case FlightPhase::ARMED:   return "ARMED";
        case FlightPhase::BOOST:   return "BOOST";
        case FlightPhase::COAST:   return "COAST";
        case FlightPhase::DESCENT: return "DESCENT";
        case FlightPhase::LANDED:  return "LANDED";
        default:                   return "UNKNOWN";
    }
}
