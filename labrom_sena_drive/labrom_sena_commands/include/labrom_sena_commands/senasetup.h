/*************************************************************************
*   Sena configuration and dimensions
*   
*   This file is part of labrom_sena_command
*
*   labrom_sena_command is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   labrom_sena_command is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with labrom_sena_command.  If not, see <http://www.gnu.org/licenses/>.
***************************************************************************/
#ifndef SENASETUP_H_
#define SENASETUP_H_

float MaxSteerDegrees = 534;



// limit steering wheel position value to normal range
inline float steer_limit_travel(float position)
{
  if (position > MaxSteerDegrees)
    position = MaxSteerDegrees;
  else if (position < -MaxSteerDegrees)
    position = -MaxSteerDegrees;
  return position;
}


#endif