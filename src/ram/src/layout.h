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

#pragma once

#include <memory>

#include "odb/db.h"

namespace ram {

// Constants
constexpr int DBU_PER_MICRON = 1000;  // 1000 database units = 1 micron

////////////////////////////////////////////////////////////////

class Layout;

class Element
{
 public:
  Element(odb::dbInst* inst);
  Element(std::unique_ptr<Layout> layout);

  // Return the bbox of the positioned element
  odb::Rect position(odb::Point origin);

 private:
  odb::dbInst* inst_ = nullptr;
  std::unique_ptr<Layout> layout_;
  odb::Point positioned_origin_;  // Store the origin used for positioning
};

class Layout
{
 public:
  // Constructor now takes both offset and spacing in microns
  Layout(odb::Orientation2D orientation, 
         double x_offset_microns = 0.0, double y_offset_microns = 0.0,
         double x_spacing_microns = 0.0, double y_spacing_microns = 0.0);

  void setOffset(double x_offset_microns, double y_offset_microns);
  void setSpacing(double x_spacing_microns, double y_spacing_microns);
  
  double getXOffsetMicrons() const { return x_offset_ / static_cast<double>(DBU_PER_MICRON); }
  double getYOffsetMicrons() const { return y_offset_ / static_cast<double>(DBU_PER_MICRON); }
  double getXSpacingMicrons() const { return x_spacing_ / static_cast<double>(DBU_PER_MICRON); }
  double getYSpacingMicrons() const { return y_spacing_ / static_cast<double>(DBU_PER_MICRON); }

  void addElement(std::unique_ptr<Element> element);

  // Return the bbox of the positioned layout
  odb::Rect position(odb::Point origin);

 private:
  odb::Orientation2D orientation_;
  std::vector<std::unique_ptr<Element>> elements_;
  int x_offset_ = 0;    // in database units
  int y_offset_ = 0;    // in database units
  int x_spacing_ = 0;   // in database units
  int y_spacing_ = 0;   // in database units
};

}  // namespace ram
