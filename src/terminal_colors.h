/*
 * terminal_colors.h
 *
 *  Created on: 12.03.2009
 *      Author: Steffen Planthaber
 *
 *      http://www.termsys.demon.co.uk/vtansi.htm


    Set Attribute Mode	<ESC>[{attr1};...;{attrn}m

    Sets multiple display attribute settings. The following lists standard attributes:

    0	Reset all attributes
    1	Bright
    2	Dim
    4	Underscore
    5	Blink
    7	Reverse
    8	Hidden

    	Foreground Colours
    30	Black
    31	Red
    32	Green
    33	Yellow
    34	Blue
    35	Magenta
    36	Cyan
    37	White

    	Background Colours
    40	Black
    41	Red
    42	Green
    43	Yellow
    44	Blue
    45	Magenta
    46	Cyan
    47	White

 *
 *
 */

#ifndef COLORS_H_
#define COLORS_H_

// colors
#define COLOR_ESC          "\033"
#define COLOR_NORMAL       COLOR_ESC"[0m"
#define COLOR_BIG          COLOR_ESC"[1;1m" //big
#define COLOR_LIGHT        COLOR_ESC"[1;2m" //italic?
#define COLOR_UNDERLINE    COLOR_ESC"[1;4m" //underscored
#define COLOR_BG_BLACK     COLOR_ESC"[1;7m" //background black, foreground white(?)
#define COLOR_BG_WHITE     COLOR_ESC"[1;8m" //background white, foreground white(?)
#define COLOR_SCORED       COLOR_ESC"[1;4m" //scored
// font colors
#define COLOR_FG_DARKGREY   COLOR_ESC"[1;30m"
#define COLOR_FG_DARKRED    COLOR_ESC"[1;31m"
#define COLOR_FG_DARKGREEN  COLOR_ESC"[1;32m"
#define COLOR_FG_DARKYELLOW COLOR_ESC"[1;33m"
#define COLOR_FG_DARKBLUE   COLOR_ESC"[1;34m"
#define COLOR_FG_DARKVIOLET COLOR_ESC"[1;35m"
#define COLOR_FG_LIGHTBLUE  COLOR_ESC"[1;36m"
#define COLOR_FG_WHITE      COLOR_ESC"[1;37m"
#define COLOR_FG_BLACK      COLOR_ESC"[1;38m"

#define COLOR_FG_MIDGREY     COLOR_ESC"[1;90m"
#define COLOR_FG_LIGHTRED    COLOR_ESC"[1;91m"
#define COLOR_FG_LIGHTGREEN  COLOR_ESC"[1;92m"
#define COLOR_FG_LIGHTYELLOW COLOR_ESC"[1;93m"
#define COLOR_FG_LIGHTVIOLET COLOR_ESC"[1;95m"

// background colors
#define COLOR_BG_DARKGREY   COLOR_ESC"[1;40m"
#define COLOR_BG_DARKRED    COLOR_ESC"[1;41m"
#define COLOR_BG_DARKGREEN  COLOR_ESC"[1;42m"
#define COLOR_BG_DARKYELLOW COLOR_ESC"[1;43m"
#define COLOR_BG_DARKBLUE   COLOR_ESC"[1;44m"
#define COLOR_BG_DARKVIOLET COLOR_ESC"[1;45m"
#define COLOR_BG_DARKBLUE2   COLOR_ESC"[1;46m"
#define COLOR_BG_LIGHTGREY  COLOR_ESC"[1;47m"

#define COLOR_BG_MIDGREY     COLOR_ESC"[1;100m"
#define COLOR_BG_LIGHTRED    COLOR_ESC"[1;101m"
#define COLOR_BG_LIGHTGREEN  COLOR_ESC"[1;102m"
#define COLOR_BG_LIGHTYELLOW COLOR_ESC"[1;103m"
#define COLOR_BG_LIGHTBLUE   COLOR_ESC"[1;104m"
#define COLOR_BG_LIGHTVIOLET COLOR_ESC"[1;105m"
#define COLOR_BG_LIGHTBLUE2   COLOR_ESC"[1;106m"



#endif

