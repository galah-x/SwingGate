// #define FRONTGATE

#define BACKGATE

#define NODEID        14    // backgate unique for each node on same network


// the battery adc sense uses a 404k resistor and 100k, for attenuation of 0.198.
// Alternatively using the 3v3 rail as reference (its a small switcher off 12)
// V = count /1024 * (3.3 / 0.1984)  or V = count * 0.01624
#define BATT_GAIN 0.01624

// these were the values used at 12V
// #define slow_open_bemf_min_val     1950
// #define slow_open_current_max_val  1000
// #define slow_close_bemf_min_val     1950
// #define slow_close_current_max_val  1000

// its 4S4P LiPo this week
#define slow_open_bemf_min_val      2300
#define slow_open_current_max_val   500
#define slow_close_bemf_min_val     2300
#define slow_close_current_max_val  500


#define fast_open_bemf_min_val      2500
#define fast_open_current_max_val   1800
#define fast_close_bemf_min_val     2500
#define fast_close_current_max_val  1800

