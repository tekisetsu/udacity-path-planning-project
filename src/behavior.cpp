
using namespace std;

double const MIN_LAN_GAP = 30.0;

bool canGoToLane(vector<vector<double>> lane_predictions, int prevision_time_index, double end_path_s) {
  bool there_is_a_gap = true;
  for(auto car_prediction : lane_predictions) {
    if( abs(end_path_s - car_prediction[prevision_time_index+1]) >  MIN_LAN_GAP){
      there_is_a_gap = true;
    } else {
      there_is_a_gap = false;
      break;
    }
  }
  return there_is_a_gap;
}






