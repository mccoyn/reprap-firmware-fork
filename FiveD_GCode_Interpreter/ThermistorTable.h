#ifndef THERMISTORTABLE_H_
#define THERMISTORTABLE_H_

#if USE_EXTRUDER_CONTROLLER == false

// Uncomment the next line if you are using a thermistor; leave it if you have a thermocouple
#define USE_THERMISTOR

// How many temperature samples to take for an average.  each sample takes about 100 usecs.
#define TEMPERATURE_SAMPLES 5

// How accurately do we maintain the temperature?
#define HALF_DEAD_ZONE 1

// Thermistor lookup table for RepRap Temperature Sensor Boards (http://make.rrrf.org/ts)
// Table derived from table in data sheet http://www.epcos.com/inf/50/db/ntc_06/GESensors__B57560__G560.pdf
// r2: 4650
// max adc: 1022
//
short temptable[][2] = {
{	30	,	300	},	//	0.139
{	39	,	280	},	//	0.1814
{	51	,	260	},	//	0.2415
{	68	,	240	},	//	0.3286
{	92	,	220	},	//	0.4581
{	127	,	200	},	//	0.6559
{	177	,	180	},	//	0.968
{	248	,	160	},	//	1.479
{	293	,	150	},	//	1.853
{	345	,	140	},	//	2.348
{	403	,	130	},	//	3.009
{	435	,	125	},	//	3.417
{	468	,	120	},	//	3.893
{	537	,	110	},	//	5.112
{	609	,	100	},	//	6.8
{	681	,	90	},	//	9.177
{	749	,	80	},	//	12.58
{	810	,	70	},	//	17.52
{	863	,	60	},	//	24.88
{	907	,	50	},	//	36.03
{	942	,	40	},	//	53.27
{	968	,	30	},	//	80.57
{	979	,	25	},	//	100
{	987	,	20	},	//	124.9
{	1001	,	10	},	//	199
{	1010	,	0	}	//	326.5

};
#define NUMTEMPS (sizeof(temptable) / sizeof(temptable[0]))

#endif
#endif
