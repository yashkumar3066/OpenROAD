///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License ... (rest of license header) ...
///////////////////////////////////////////////////////////////////////////////

%{
#include "ord/OpenRoad.hh"
#include "ram/ram.h"
#include "utl/Logger.h"
#include "sta/FuncExpr.hh"
#include "sta/Liberty.hh"
#include "sta/PortDirection.hh"
#include "db_sta/dbSta.hh"
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
 * @param buf_cell_name Name of the buffer cell. // New parameter
 * @param read_ports Number of read ports.
 * @param mask Flag to enable masking.
 */
void
generate_ram_netlist_cmd(int bytes_per_word,
                         int word_count,
                         const char* storage_cell_name,
                         const char* tristate_cell_name,
                         const char* inv_cell_name,
                         const char* buf_cell_name, // New parameter
                         const int read_ports,
                         bool mask)
{
  auto* app = ord::OpenRoad::openRoad();
  auto* ram_gen = app->getRamGen();
  auto* db = app->getDb();
  auto* sta = app->getSta();
  auto* logger = app->getLogger(); // Get logger for error messages

  ram_gen->init(db, sta->getDbNetwork(), logger);

  if (word_count != 4 && word_count != 8) {
    logger->error(utl::RAM, 21, "Only 4 or 8 words are supported.");
    return;
  }

  // Find required cells
  odb::dbMaster* storage_cell = db->findMaster(storage_cell_name);
  if (!storage_cell) {
    logger->error(utl::RAM, 11, "Storage cell {} can't be found", storage_cell_name);
    return;
  }

  odb::dbMaster* tristate_cell = db->findMaster(tristate_cell_name);
  if (!tristate_cell) {
    logger->error(utl::RAM, 7, "Tristate cell {} can't be found", tristate_cell_name);
    return;
  }

  odb::dbMaster* inv_cell = db->findMaster(inv_cell_name);
  if (!inv_cell) {
    logger->error(utl::RAM, 19, "Inverter cell {} can't be found", inv_cell_name);
    return;
  }

  // Find the buffer cell
  odb::dbMaster* buf_cell = db->findMaster(buf_cell_name);
  if (!buf_cell) {
    logger->error(utl::RAM, 106, "Buffer cell {} can't be found", buf_cell_name); // New error code
    return;
  }


  // Call generate with the correct parameters, including buf_cell
  ram_gen->generate_ram_netlist(bytes_per_word, word_count, read_ports,
                    storage_cell, tristate_cell, inv_cell, buf_cell, mask);
}

%} // inline
