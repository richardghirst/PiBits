//$Id: clock.h 491 2005-12-23 11:01:49Z murrayc $ -*- c++ -*-

/* gtkmm example Copyright (C) 2002 gtkmm development team
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef GTKMM_EXAMPLE_CUBE_H
#define GTKMM_EXAMPLE_CUBE_H

#include <gtkmm/drawingarea.h>

class Cube : public Gtk::DrawingArea
{
public:
  Cube();
  virtual ~Cube();

protected:
  //Override default signal handler:
  virtual bool on_draw(const Cairo::RefPtr<Cairo::Context>& cr);

  bool on_timeout();

  double m_radius;
  double m_line_width;

private:
  void calc_scale(const Cairo::RefPtr<Cairo::Context>& cr);
};

#endif // GTKMM_EXAMPLE_CUBE_H
