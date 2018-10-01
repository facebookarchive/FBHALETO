// Copyright (C) 2018-present, Facebook, Inc.
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; version 2 of the License.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//

#include <stdio.h>
#include <time.h>
#include "date_vec.h"

int get_year(time_t t) {
    struct tm ts = *gmtime(&t);
    return ts.tm_year+1900;
}

int get_month(time_t t) {
    struct tm ts = *gmtime(&t);
    return ts.tm_mon+1;
}

int get_day(time_t t) {
    struct tm ts = *gmtime(&t);
    return ts.tm_mday;
}

int get_hour(time_t t) {
    struct tm ts = *gmtime(&t);
    return ts.tm_hour;
}

int get_min(time_t t) {
    struct tm ts = *gmtime(&t);
    return ts.tm_min;
}

int get_sec(time_t t) {
    struct tm ts = *gmtime(&t);
    return ts.tm_sec;
}

int *date_vec(time_t t) {
    struct tm ts = *gmtime(&t);

    int y  = ts.tm_year +1900;
    int mo = ts.tm_mon  +1;
    int d  = ts.tm_mday;
    int h  = ts.tm_hour;
    int mi = ts.tm_min;
    int s  = ts.tm_sec;

    static int ymdhms[6];

    ymdhms[0] = y;
    ymdhms[1] = mo;
    ymdhms[2] = d;
    ymdhms[3] = h;
    ymdhms[4] = mi;
    ymdhms[5] = s;

    return ymdhms;
}
