/* Copying and distribution of this file, with or without modification,
 * are permitted in any medium without royalty. This file is offered as-is,
 * without any warranty.
 */

/*! @file process_frame.c
 * @brief Contains the actual algorithm and calculations.
 */

/* Definitions specific to this application. Also includes the Oscar main header file. */
#include "template.h"
#include <string.h>
#include <stdlib.h>

#define IMG_SIZE NUM_COLORS*(OSC_CAM_MAX_IMAGE_WIDTH/2)*(OSC_CAM_MAX_IMAGE_HEIGHT/2)

const int nc = OSC_CAM_MAX_IMAGE_WIDTH/2;
const int nr = OSC_CAM_MAX_IMAGE_HEIGHT/2;

int k = 5;
int TextColor;
int avgDxy[3][IMG_SIZE];
int const Border = 10;
int helpBuf[IMG_SIZE];
int totalMax = 0;


void ResetProcess()
{
	//called when "reset" button is pressed
	if(TextColor == CYAN)
		TextColor = MAGENTA;
	else
		TextColor = CYAN;
}


void ProcessFrame()
{
	uint32 t1, t2;
	//char Text[] = "hallo world";
	//initialize counters

	if(data.ipc.state.nStepCounter == 1) {
		//use for initialization; only done in first step
		memset(data.u8TempImage[THRESHOLD], 0, IMG_SIZE);
		TextColor = CYAN;
	} else {
		//example for time measurement
		t1 = OscSupCycGet();
		CalcDeriv();
		AvgDeriv(0);
		AvgDeriv(1);
		AvgDeriv(2);
		CalcMc();
		LocalMax();
		//example for copying sensor image to background image
		//memcpy(data.u8TempImage[BACKGROUND], data.u8TempImage[SENSORIMG], IMG_SIZE);
		//example for time measurement
		t2 = OscSupCycGet();

		//example for log output to console
		OscLog(INFO, "required = %d us\n", OscSupCycToMicroSecs(t2-t1));

		//example for drawing output
		//draw line
		//DrawLine(10, 100, 200, 20, RED);
		//draw open rectangle
		//DrawBoundingBox(20, 10, 50, 40, false, GREEN);
		//draw filled rectangle
		//DrawBoundingBox(80, 100, 110, 120, true, BLUE);
		//DrawString(200, 200, strlen(Text), TINY, TextColor, Text);

	}
}
int dx = 0;
int dy = 0;
void CalcDeriv() {
	int c, r;
	for (r = nc; r < nr * nc - nc; r += nc) {/* we skip the first and last line */
		for (c = 1; c < nc - 1; c++) {
			/* do pointer arithmetics with respect to center pixel location */
			unsigned char* p = &data.u8TempImage[SENSORIMG][r + c];
			/* implement Sobel filter */
			dx = -(int) *(p - nc - 1) + (int) *(p - nc + 1)
					- 2 * (int) *(p - 1) + 2 * (int) *(p + 1)
					- (int) *(p + nc - 1) + (int) *(p + nc + 1);
			dy = -(int) *(p - nc - 1) - 2 * (int) *(p - nc)
					- (int) *(p - nc + 1) + (int) *(p + nc - 1)
					+ 2 * (int) *(p + nc) + (int) *(p + nc + 1);
			avgDxy[0][r + c] = dx * dx;
			avgDxy[1][r + c] = dy * dy;
			avgDxy[2][r + c] = dx * dy;
			/*data.u8TempImage[THRESHOLD][r + c] = (uint8) MIN(255,
					MAX(0, (dx*dx) >> 10));*/
			/*data.u8TempImage[BACKGROUND][r + c] = (uint8) MIN(255,
								MAX(0, (dx*dy) >> 10));*/
			/*data.u8TempImage[BACKGROUND][r + c] = (uint8) MIN(255,
					MAX(0, 128+dx));
			data.u8TempImage[THRESHOLD][r + c] = (uint8) MIN(255,
					MAX(0, 128+dy));*/
		}
	}
}


void CalcMc(){

	int r;
	int c;

	for (r = nc*(Border+1); r < nr * nc - nc*(Border+1); r += nc) {/* we skip first and last lines (empty) */
				for (c = Border + 1; c < nc - (Border + 1); c++) {/* +1 because we have one empty border column */
				int dx2 = (avgDxy[0][r + c] >> 7);
					int dy2 = (avgDxy[1][r + c] >> 7);
					int dxy2 = (avgDxy[2][r + c] >> 7);
	int mc = ((dx2*dy2) - dxy2*dxy2)-((k*(dxy2+dxy2)*(dxy2+dxy2))>>7);
	avgDxy[0][r + c] = mc;
	//get global Max
	if(mc>totalMax){
		totalMax = mc;
	}
	//data.u8TempImage[BACKGROUND][r + c] = (uint8) MIN(255, MAX(0, avgDxy[0][r+c] >> 10));
			}
	}
}


void AvgDeriv(int Index) {
//do average in x-direction
	int c, r;
	for (r = nc; r < nr * nc - nc; r += nc) {/* we skip first and last lines (empty) */
		for (c = Border + 1; c < nc - (Border + 1); c++) {/* +1 because we have one empty border column */
			/* do pointer arithmetics with respect to center pixel location */
			int* p = &avgDxy[Index][r + c];
			int sx =   (*(p - 6) + *(p + 6))
					+ ((*(p - 5) + *(p + 5)) << 2)
					+ ((*(p - 4) + *(p + 4)) << 3)
					+ ((*(p - 3) + *(p + 3)) << 5)
					+ ((*(p - 2) + *(p + 2)) << 6)
					+ ((*(p - 1) + *(p + 1)) << 6) + (*p << 7);
//now averaged
			helpBuf[r + c] = (sx >> 8);
		}
	}

	for (r = nc*(Border+1); r < nr * nc - nc*(Border+1); r += nc) {/* we skip first and last lines (empty) */
			for (c = Border + 1; c < nc - (Border + 1); c++) {/* +1 because we have one empty border column */
				/* do pointer arithmetics with respect to center pixel location */
				int* p = &helpBuf[r + c];
				int sy =   (*(p - nc * 6) + *(p + nc * 6))
						+ ((*(p - nc * 5) + *(p + nc * 5)) << 2)
						+ ((*(p - nc * 4) + *(p + nc * 4)) << 3)
						+ ((*(p - nc * 3) + *(p + nc * 3)) << 5)
						+ ((*(p - nc * 2) + *(p + nc * 2)) << 6)
						+ ((*(p - nc) + *(p + nc)) << 6) + (*p << 7);
				//do average in y-direction
				avgDxy[Index][r+c] = (sy >> 8);

				/*data.u8TempImage[BACKGROUND][r + c] = (uint8) MIN(255,
									MAX(0, avgDxy[Index][r+c] >> 11));*/
			}
		}

}

void FindMaximas(){
	int c, r;
	int Index = 0;
	int SizeBox = 20;
	for (r = nc*(Border+1); r < nr * nc - nc*(Border+1); r += nc) {/* we skip first and last lines (empty) */
					for (c = Border + 1; c < nc - (Border + 1); c++) {/* +1 because we have one empty border column */
						int* p = &avgDxy[Index][r + c];
						int* x = &avgDxy[Index][r + c];
						if((*p >= (totalMax/data.ipc.state.nThreshold)) && (*p == *x)){
							avgDxy[Index][r+c] = 0;
							DrawBoundingBox(c-SizeBox, r/nc+SizeBox, c+SizeBox, r/nc-SizeBox, false, GREEN);
						}
						else{
							avgDxy[Index][r+c] = 255;
						}
						//data.u8TempImage[BACKGROUND][r + c] = (uint8) MIN(255, MAX(0, avgDxy[Index][r+c]));

					}
	}
}




void LocalMax() {
//do average in x-direction
	memset(data.u8TempImage[THRESHOLD], 0, IMG_SIZE);
	int Index = 0;
	int c, r;
	int localMax = 0;
	int SizeBox = 3;
	for (r = nc; r < nr * nc - nc; r += nc) {/* we skip first and last lines (empty) */
		for (c = Border + 1; c < nc - (Border + 1); c++) {/* +1 because we have one empty border column */
			/* do pointer arithmetics with respect to center pixel location */
			localMax = 0;
			int* p = &avgDxy[0][r + c];
			for(int i=-Border; i<Border; i++){

				if( *(p + i)>localMax){
					localMax = *(p + i);
				}
			}

//now averaged
			helpBuf[r + c] = (localMax);

		}
	}

	for (r = nc*(Border+1); r < nr * nc - nc*(Border+1); r += nc) {/* we skip first and last lines (empty) */
			for (c = Border + 1; c < nc - (Border + 1); c++) {/* +1 because we have one empty border column */
				/* do pointer arithmetics with respect to center pixel location */
				localMax = 0;
				int* p = &helpBuf[r + c];
				for(int i=-Border; i<Border; i++){
								if(*(p + nc * i) >localMax){
									localMax = (*(p + nc * i));
								}
							}

				//do average in y-direction
				if(avgDxy[Index][r+c] == (localMax) && localMax > (totalMax*data.ipc.state.nThreshold)>>7){
					data.u8TempImage[THRESHOLD][r + c] = (uint8) MIN(255,
														MAX(0, 255));
					DrawBoundingBox(c-SizeBox, r/nc+SizeBox, c+SizeBox, r/nc-SizeBox, false, GREEN);
				}
			}
		}

}





