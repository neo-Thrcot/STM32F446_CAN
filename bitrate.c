#include <stdio.h>
#include <stdint.h>

#define APB1CLK				45000000
#define ABS_DIFF(a, b)		((a > b) ? a - b : b - a)
#define CAN_RATE(TS1, TS2, BRP)			((double)APB1CLK / ((TS1 + TS2 + 3) * (BRP + 1)))
#define CAN_SP(TS1, TS2)				(((double)(TS1 + 2) / (TS1 + TS2 + 3)) * 100.0)

typedef struct
{
	uint8_t TS1;
	uint8_t	TS2;
	uint8_t SJW;
	uint16_t BRP;
	uint64_t total_tq;
} CAN_BaudRateVal_t;

CAN_BaudRateVal_t calcRateVal(uint64_t bitrate);

int main(void)
{
    CAN_BaudRateVal_t rate = calcRateVal(500000);

    if(rate.BRP == 0 && rate.TS1 == 0 && rate.TS2 == 0 && rate.SJW == 0 && rate.total_tq == 0) {
        printf("Not pattern found!\n");
    } else {
        printf("BRP: %u, TS1: %u, TS2: %u, SJW: %u, TotalTQ: %llu\n",
               rate.BRP+1, rate.TS1+1, rate.TS2+1, rate.SJW+1, rate.total_tq);
        printf("Calculated Rate: %.2f bps\n", CAN_RATE(rate.TS1, rate.TS2, rate.BRP));
        printf("Sample Point: %.2f%%\n", CAN_SP(rate.TS1, rate.TS2));
    }
    
    return 0;
}

CAN_BaudRateVal_t calcRateVal(uint64_t bitrate)
{
	uint64_t calc_rate = 0;
	uint64_t rate_error = 0;
	uint32_t pattern_cnt = 0;
	uint8_t allow_flag = 0;
	double sample_point = 0.0;

	CAN_BaudRateVal_t rate_pattern1 = {0, 0, 0, 0, 0};
	CAN_BaudRateVal_t rate_pattern2 = {0, 0, 0, 0, 0};

	for(uint16_t brp = 0; brp < 1024; brp++) {
		for(uint8_t ts1 = 0; ts1 < 16; ts1++) {
			for(uint8_t ts2 = 0; ts2 < 8 && ts2 <= ts1; ts2++) {
				calc_rate = CAN_RATE(ts1, ts2, brp);
				rate_error = ABS_DIFF(calc_rate, bitrate);

				if(rate_error == 0) {
					sample_point = CAN_SP(ts1, ts2);

					if(sample_point >= 75.0 && sample_point <= 85.0) {
						rate_pattern1.BRP = brp;
						rate_pattern1.TS1 = ts1;
						rate_pattern1.TS2 = ts2;
						rate_pattern1.total_tq = ts1 + ts2 + 3;
						pattern_cnt++;

						if(allow_flag == 0) {
							rate_pattern2 = rate_pattern1;
							allow_flag = 1;
						} else if(rate_pattern1.total_tq > rate_pattern2.total_tq) {
							rate_pattern2 = rate_pattern1;
						} else if(rate_pattern1.total_tq == rate_pattern2.total_tq 
								  && CAN_SP(rate_pattern1.TS1, rate_pattern1.TS2) > CAN_SP(rate_pattern2.TS1, rate_pattern2.TS2)) {
							rate_pattern2 = rate_pattern1;
						}
					}
				}
			}
		}
	}

	if(rate_pattern2.TS2 < 4) {
		rate_pattern2.SJW = rate_pattern2.TS2;
	} else {
		rate_pattern2.SJW = 3;
	}

	return rate_pattern2;
}