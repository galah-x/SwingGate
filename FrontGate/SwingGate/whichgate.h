#define FRONTGATE

#define NODEID        9    //frontgate unique for each node on same network

// limit switches currently fitted to front gate only.
#define PROXIMITY_FITTED

// #define BACKGATE

// the battery adc sense uses a 404k resistor and 100k, for attenuation of 0.198.
// Alternatively using the 3v3 rail as reference (its a small switcher off 12)
// V = count /1024 * (3.3 / 0.1984)  or V = count * 0.01624
#define BATT_GAIN 0.01624

// frontgate
#define slow_open_bemf_min_val     1500
#define slow_close_bemf_min_val    1200
#define fast_open_bemf_min_val     2500
#define fast_close_bemf_min_val    2000
#define slow_open_current_max_val  900
#define slow_close_current_max_val 1000
#define fast_open_current_max_val  1450
#define fast_close_current_max_val 1750
