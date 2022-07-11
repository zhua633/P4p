#pragma once

void datalogClear();
void datalogDataGroupStart();
void datalogAddValue(int Col, float data);
void datalogDataGroupEnd();
void datalogClose();
