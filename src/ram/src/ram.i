///////////////////////////////////////////////////////////////////////////////
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
///////////////////////////////////////////////////////////////////////////////


%{
#include "ord/OpenRoad.hh"
#include "ram/ram.h"
#include "utl/Logger.h"
#include "sta/FuncExpr.hh"
#include "sta/Liberty.hh"
#include "sta/PortDirection.hh"
#include "db_sta/dbSta.hh"  // Include this header for the full definition of sta::dbSta
%}

%include "../../Exception.i"

%inline %{

/**
 * @brief Generates the RAM netlist based on specified parameters.
 * @param bytes_per_word Number of bytes per word.
 * @param word_count Number of words.
 * @param storage_cell_name Name of the storage cell.
 * @param tristate_cell_name Name of the tristate cell.
 * @param inv_cell_name Name of the inverter cell.
 * @param read_ports Number of read ports.
 * @param mask Flag to enable masking.
 */
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
  auto* sta = app->getSta();

  ram_gen->init(db, sta->getDbNetwork(), app->getLogger());

  // Check that word_count is either 4 or 8
  if (word_count != 4 && word_count != 8) {
    app->getLogger()->error(utl::RAM,
                            21,
                            "Only 4 or 8 words are supported.");
    return;
  }

  // Find the required cells (storage, tristate, and inv)
  odb::dbMaster* storage_cell = nullptr;
  if (storage_cell_name[0] != '\0') {
    storage_cell = db->findMaster(storage_cell_name);
    if (!storage_cell) {
      app->getLogger()->error(utl::RAM,
                              11,
                              "Storage cell {} can't be found",
                              storage_cell_name);
      return;
    }
  } else {
    app->getLogger()->error(utl::RAM,
                            14,
                            "Storage cell name must be provided.");
    return;
  }

  odb::dbMaster* tristate_cell = nullptr;
  if (tristate_cell_name[0] != '\0') {
    tristate_cell = db->findMaster(tristate_cell_name);
    if (!tristate_cell) {
      app->getLogger()->error(utl::RAM,
                              7,
                              "Tristate cell {} can't be found",
                              tristate_cell_name);
      return;
    }
  } else {
    app->getLogger()->error(utl::RAM,
                            15,
                            "Tristate cell name must be provided.");
    return;
  }

  odb::dbMaster* inv_cell = nullptr;
  if (inv_cell_name[0] != '\0') {
    inv_cell = db->findMaster(inv_cell_name);
    if (!inv_cell) {
      app->getLogger()->error(utl::RAM,
                              19,
                              "Inverter cell {} can't be found",
                              inv_cell_name);
      return;
    }
  } else {
    app->getLogger()->error(utl::RAM,
                            20,
                            "Inverter cell name must be provided.");
    return;
  }

  // Call generate with the correct parameters
  ram_gen->generate_ram_netlist(bytes_per_word, word_count, read_ports,
                    storage_cell, tristate_cell, inv_cell, mask);
}

%} // inline
