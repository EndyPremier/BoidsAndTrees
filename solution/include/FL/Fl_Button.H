//
// "$Id: Fl_Button.H,v 1.5.2.2 2000/06/05 21:20:20 mike Exp $"
//
// Button header file for the Fast Light Tool Kit (FLTK).
//
// Copyright 1998-2000 by Bill Spitzak and others.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Library General Public
// License as published by the Free Software Foundation; either
// version 2 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Library General Public License for more details.
//
// You should have received a copy of the GNU Library General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
// USA.
//
// Please report all bugs and problems to "fltk-bugs@fltk.org".
//

#ifndef Fl_Button_H
#define Fl_Button_H

#ifndef Fl_Widget_H
#include "Fl_Widget.H"
#endif

// values for type()
#define FL_TOGGLE_BUTTON	1
#define FL_RADIO_BUTTON		(FL_RESERVED_TYPE+2)
#define FL_HIDDEN_BUTTON	3 // for Forms compatability

extern FL_EXPORT int fl_old_shortcut(const char*);

class Fl_Button : public Fl_Widget {

  int shortcut_;
  char value_;
  char oldval;
  uchar down_box_;

protected:

  virtual FL_EXPORT void draw();

public:

  virtual FL_EXPORT int handle(int);
  FL_EXPORT Fl_Button(int,int,int,int,const char * = 0);
  FL_EXPORT int value(int);
  char value() const {return value_;}
  int set() {return value(1);}
  int clear() {return value(0);}
  FL_EXPORT void setonly(); // this should only be called on FL_RADIO_BUTTONs
  int shortcut() const {return shortcut_;}
  void shortcut(int s) {shortcut_ = s;}
  Fl_Boxtype down_box() const {return (Fl_Boxtype)down_box_;}
  void down_box(Fl_Boxtype b) {down_box_ = b;}

  // back compatability:
  void shortcut(const char *s) {shortcut(fl_old_shortcut(s));}
  Fl_Color down_color() const {return selection_color();}
  void down_color(uchar c) {selection_color(c);}
};

#endif

//
// End of "$Id: Fl_Button.H,v 1.5.2.2 2000/06/05 21:20:20 mike Exp $".
//
