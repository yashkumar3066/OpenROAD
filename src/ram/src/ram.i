//////////////////////////////////////////////////////////////////////////////
//
// BSD 3-Clause License
//
// Copyright (c) 2023, Precision Innovations Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
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
%{
#include "ord/OpenRoad.hh"
#include "ram/ram.h"
#include "utl/Logger.h"
#include "sta/FuncExpr.hh"
#include "sta/Liberty.hh"
#include "sta/PortDirection.hh" // Include this header for sta::PortDirection
%}

%include "../../Exception.i"

%inline %{

void
generate_ram_netlist_cmd(int bytes_per_word,
                         int word_count,
                         const char* storage_cell_name,
                         const char* tristate_cell_name,
                         const char* inv_cell_name,
                         const int read_ports,
                         bool mask)
{
  auto* app = ord::OpenRoad::openRoad();
  auto* ram_gen = app->getRamGen();
  auto* db = app->getDb();

  // Find the required cells (storage, tristate, and inv)
  odb::dbMaster* storage_cell = nullptr;
  if (storage_cell_name[0] != '\0') {
    storage_cell = db->findMaster(storage_cell_name);
    if (!storage_cell) {
      app->getLogger()->error(utl::RAM,
                              4,
                              "Storage cell {} can't be found",
                              storage_cell_name);
    }
  }

  odb::dbMaster* tristate_cell = nullptr;
  if (tristate_cell_name[0] != '\0') {
    tristate_cell = db->findMaster(tristate_cell_name);
    if (!tristate_cell) {
      app->getLogger()->error(utl::RAM,
                              7,
                              "Tristate cell {} can't be found",
                              tristate_cell_name);
    }
  }

  odb::dbMaster* inv_cell = nullptr;
  if (inv_cell_name[0] != '\0') {
    inv_cell = db->findMaster(inv_cell_name);
    if (!inv_cell) {
      app->getLogger()->error(utl::RAM,
                              8,
                              "Inverter cell {} can't be found",
                              inv_cell_name);
    }
  }

  // Find required master cells for logic gates (AND, Clock Gate)
  ram_gen->findMasters();

  // Find AND gates (2-input, 3-input, etc.)
  odb::dbMaster* and2_cell = ram_gen->findMaster(
      [](sta::LibertyPort* port) {
        if (!port->direction()->isOutput()) {
          return false;
        }
        auto function = port->function();
        return function && function->op() == sta::FuncExpr::op_and
               && function->left()->op() == sta::FuncExpr::op_port
               && function->right()->op() == sta::FuncExpr::op_port;
      },
      "and2");

  // If more input AND gates are needed for the decoder, you can find them similarly
  odb::dbMaster* and3_cell = ram_gen->findMaster(
      [](sta::LibertyPort* port) {
        if (!port->direction()->isOutput()) {
          return false;
        }
        auto function = port->function();
        return function && function->op() == sta::FuncExpr::op_and;
      },
      "and3");

  odb::dbMaster* clock_gate_cell = ram_gen->findMaster(
      [](sta::LibertyPort* port) {
        return port->libertyCell()->isClockGate();
      },
      "clock gate");

  // Error handling for missing cells
  if (!and2_cell) {
    app->getLogger()->error(utl::RAM,
                            11,
                            "AND2 gate cell can't be found");
  }
  if (!clock_gate_cell) {
    app->getLogger()->error(utl::RAM,
                            13,
                            "Clock gate cell can't be found");
  }

  // Now call generate with the correct parameters, passing all the necessary cells
  ram_gen->generate(bytes_per_word, word_count, read_ports,
                    storage_cell, tristate_cell, inv_cell, mask);
}

%} // inline

