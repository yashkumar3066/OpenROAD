/////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2023, Precision Innovations Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
///////////////////////////////////////////////////////////////////////////////

#include "layout.h"

#include "ram/ram.h"
#include "utl/Logger.h"

namespace ram {

using odb::dbBTerm;
using odb::dbInst;
using odb::dbNet;
using odb::Point;
using odb::Rect;

using utl::RAM;

////////////////////////////////////////////////////////////////

Element::Element(odb::dbInst* inst) : inst_(inst)
{
}

Element::Element(std::unique_ptr<Layout> layout) : layout_(std::move(layout))
{
}

Rect Element::position(Point origin)
{
  positioned_origin_ = origin;  // Store the origin used for positioning
  if (inst_) {
    inst_->setLocation(origin.getX(), origin.getY());
    inst_->setPlacementStatus(odb::dbPlacementStatus::PLACED);
    return inst_->getBBox()->getBox();
  }

  return layout_->position(origin);
}

////////////////////////////////////////////////////////////////

Layout::Layout(odb::Orientation2D orientation, 
               double x_offset_microns, double y_offset_microns,
               double x_spacing_microns, double y_spacing_microns)
    : orientation_(orientation),
      x_offset_(static_cast<int>(x_offset_microns * DBU_PER_MICRON)),
      y_offset_(static_cast<int>(y_offset_microns * DBU_PER_MICRON)),
      x_spacing_(static_cast<int>(x_spacing_microns * DBU_PER_MICRON)),
      y_spacing_(static_cast<int>(y_spacing_microns * DBU_PER_MICRON))
{
}

void Layout::setOffset(double x_offset_microns, double y_offset_microns)
{
  x_offset_ = static_cast<int>(x_offset_microns * DBU_PER_MICRON);
  y_offset_ = static_cast<int>(y_offset_microns * DBU_PER_MICRON);
}

void Layout::setSpacing(double x_spacing_microns, double y_spacing_microns)
{
  x_spacing_ = static_cast<int>(x_spacing_microns * DBU_PER_MICRON);
  y_spacing_ = static_cast<int>(y_spacing_microns * DBU_PER_MICRON);
}

Rect Layout::position(Point origin)
{
  // Apply offset to the origin before positioning elements
  origin = Point(origin.getX() + x_offset_, origin.getY() + y_offset_);
  
  Rect bbox(origin, origin);
  if (orientation_ == odb::horizontal) {
    for (auto& elem : elements_) {
      auto bounds = elem->position(origin);
      bbox.merge(bounds);
      // Move origin to the right edge of current element plus spacing
      origin = Point(bounds.xMax() + x_spacing_, origin.getY());
    }
  } else {
    for (auto& elem : elements_) {
      auto bounds = elem->position(origin);
      bbox.merge(bounds);
      // Move origin to the top edge of current element plus spacing
      origin = Point(origin.getX(), bounds.yMax() + y_spacing_);
    }
  }
  return bbox;
}

void Layout::addElement(std::unique_ptr<Element> element)
{
  elements_.push_back(std::move(element));
}

}  // namespace ram
