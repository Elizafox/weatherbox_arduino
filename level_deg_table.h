#ifndef LEVEL_DEG_TABLE_H
#define LEVEL_DEG_TABLE_H

struct level_deg
{
  unsigned int level;  // Analouge pin level
  float deg;           // Degrees of rotation of the vane
};


// Calibrated for 3.3 volts on a atmega2560.
#if 0 
static const level_deg level_deg_table[] = {
  {52, 270},
  {90, 315},
  {130, 292.5},
  {157, 0},
  {212, 337.5},
  {260, 225},
  {280, 247.5},
  {371, 45},
  {407, 22.5},
  {486, 180},
  {514, 202.5},
  {553, 135},
  {592, 157.5},
  {614, 90},
  {620, 67.5},
  {632, 112.5},
};
#endif

// Defined for a 3.3 volt board
static const level_deg level_deg_table[] = {
  {79, 270},
  {137, 315},
  {196, 292.5},
  {238, 0},
  {321, 337.5},
  {393, 225},
  {424, 247.5},
  {562, 45},
  {617, 22.5},
  {736, 180},
  {779, 202.5},
  {839, 135},
  {897, 157.5},
  {930, 90},
  {939, 67.5},
  {957, 112.5},
};

static const size_t level_deg_table_len = (sizeof(level_deg_table) /
                                           sizeof(level_deg_table[0]));

#endif // LEVEL_DEG_TABLE_H
