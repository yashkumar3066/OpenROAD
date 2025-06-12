#include "ram/ram.h"
#include "db_sta/dbNetwork.hh"
#include "sta/FuncExpr.hh"
#include "sta/Liberty.hh"
#include "sta/PortDirection.hh"
#include "utl/Logger.h"
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <vector>
#include <array>

namespace ram {

using odb::dbBlock;
using odb::dbBTerm;
using odb::dbInst;
using odb::dbMaster;
using odb::dbNet;
using utl::RAM;
using std::vector;
using std::array;

// Constructor and init remain the same...
RamGen::RamGen()
    : db_(nullptr),
      block_(nullptr),
      network_(nullptr),
      logger_(nullptr),
      storage_cell_(nullptr),
      tristate_cell_(nullptr),
      inv_cell_(nullptr),
      and2_cell_(nullptr),
      clock_gate_cell_(nullptr),
      buf_cell_(nullptr),
      gate_counter_(0),
      net_counter_(0),
      max_and_inputs_(2) {}

void RamGen::init(odb::dbDatabase* db, sta::dbNetwork* network, Logger* logger) {
  db_ = db;
  network_ = network;
  logger_ = logger;
}


// makeInst, makeNet, makeBTerm, make_bit, make_byte remain the same...
dbInst* RamGen::makeInst(
    Layout* layout,
    const std::string& prefix,
    const std::string& name,
    dbMaster* master,
    const vector<std::pair<std::string, dbNet*>>& connections) {
  const auto inst_name = fmt::format("{}.{}", prefix, name);
   if (!master) {
       logger_->error(RAM, 108, "Attempted to create instance {} with null master.", inst_name);
       return nullptr;
  }
  auto inst = dbInst::create(block_, master, inst_name.c_str());
  if (!inst) {
      logger_->error(RAM, 101, "Failed to create instance: {}", inst_name);
      return nullptr;
  }

  // Debug log before placement
  logger_->info(RAM, 140, "Before placement - Instance {}: master={}, location=({},{}), bbox=({},{})-({},{})",
               inst_name, master->getName(),
               inst->getLocation().x(), inst->getLocation().y(),
               inst->getBBox()->getBox().xMin(), inst->getBBox()->getBox().yMin(),
               inst->getBBox()->getBox().xMax(), inst->getBBox()->getBox().yMax());

  for (auto& [mterm_name, net] : connections) {
    auto mterm = master->findMTerm(mterm_name.c_str());
    if (!mterm) {
      logger_->error(RAM, 9, "MTerm {} of cell {} not found.", mterm_name, master->getName());
      continue;
    }
    auto iterm = inst->getITerm(mterm);
    if (!iterm) {
        logger_->error(RAM, 102, "ITerm for MTerm {} not found on instance {}", mterm_name, inst_name);
        continue;
    }
    if (!net) {
         logger_->error(RAM, 111, "Attempted to connect null net to MTerm {} of instance {}", mterm_name, inst_name);
         continue;
    }
    iterm->connect(net);
  }

  layout->addElement(std::make_unique<Element>(inst));

  // Debug log after placement
  logger_->info(RAM, 141, "After placement - Instance {}: location=({},{}), bbox=({},{})-({},{})",
               inst_name,
               inst->getLocation().x(), inst->getLocation().y(),
               inst->getBBox()->getBox().xMin(), inst->getBBox()->getBox().yMin(),
               inst->getBBox()->getBox().xMax(), inst->getBBox()->getBox().yMax());

  return inst;
}

dbNet* RamGen::makeNet(const std::string& prefix, const std::string& name) {
  const auto net_name = fmt::format("{}.{}", prefix, name);
  return dbNet::create(block_, net_name.c_str());
}

dbNet* RamGen::makeBTerm(const std::string& name) {
  auto net = dbNet::create(block_, name.c_str());
  dbBTerm::create(net, name.c_str());
  return net;
}

std::unique_ptr<Element> RamGen::make_bit(const std::string& prefix,
                                          const int read_ports,
                                          dbNet* clock,
                                          vector<odb::dbNet*>& select,
                                          dbNet* data_input,
                                          vector<odb::dbNet*>& data_output) {
  auto layout = std::make_unique<Layout>(odb::horizontal);
  auto storage_net = makeNet(prefix, "storage");

  makeInst(layout.get(), prefix, "bit", storage_cell_, {{"GATE", clock}, {"D", data_input}, {"Q", storage_net}});

  for (int read_port = 0; read_port < read_ports; ++read_port) {
     if (read_port >= select.size()) {
        logger_->error(RAM, 112, "Index out of bounds for select net in make_bit (read_port={})", read_port);
        return nullptr;
    }
    if (read_port >= data_output.size()) {
        logger_->error(RAM, 113, "Index out of bounds for data_output net in make_bit (read_port={})", read_port);
        return nullptr;
    }
    makeInst(layout.get(), prefix, fmt::format("obuf{}", read_port), tristate_cell_, {{"A", storage_net}, {"TE_B", select[read_port]}, {"Z", data_output[read_port]}});
  }

  return std::make_unique<Element>(std::move(layout));
}

std::unique_ptr<Element> RamGen::make_byte(
    const std::string& prefix,
    const int read_ports,
    dbNet* clock,
    dbNet* write_enable,
    const std::vector<dbNet*>& select_signals,
    const std::array<dbNet*, 8>& data_input,
    const std::vector<std::array<dbNet*, 8>>& data_output, // Byte-specific outputs
    bool mask) {
  auto layout = std::make_unique<Layout>(odb::horizontal);
  dbNet* select = select_signals[0];
  dbNet* we_net = makeNet(prefix, "we_net");
  makeInst(layout.get(), prefix, "we_and", and2_cell_, {{"A", select}, {"B", write_enable}, {"X", we_net}});
  dbNet* clock_b_net = makeNet(prefix, "clock_b");
  dbNet* gclock_net = makeNet(prefix, "gclock");
  makeInst(layout.get(), prefix, "clock_inv", inv_cell_, {{"A", clock}, {"Y", clock_b_net}});
  makeInst(layout.get(), prefix, "icg", clock_gate_cell_, {{"CLK", clock_b_net}, {"GATE", we_net}, {"GCLK", gclock_net}});

  vector<dbNet*> select_b_nets(read_ports);
  for (int i = 0; i < read_ports; ++i) {
    select_b_nets[i] = makeNet(prefix, fmt::format("select{}_b", i));
    makeInst(layout.get(), prefix, fmt::format("select_inv_{}", i), inv_cell_, {{"A", select}, {"Y", select_b_nets[i]}});
  }

  for (int bit = 0; bit < 8; ++bit) {
    vector<dbNet*> bit_outputs(read_ports);
    for (int read_port = 0; read_port < read_ports; ++read_port) {
       if (read_port < data_output.size() && bit < data_output[read_port].size()) {
            bit_outputs[read_port] = data_output[read_port][bit];
        } else {
             logger_->error(RAM, 109, "Index out of bounds accessing data_output in make_byte for prefix {} read_port {} bit {}", prefix, read_port, bit);
             return nullptr;
        }
    }
    auto bit_element = make_bit(fmt::format("{}.bit{}", prefix, bit), read_ports, gclock_net, select_b_nets, data_input[bit], bit_outputs);
    if (!bit_element) return nullptr;
    layout->addElement(std::move(bit_element));
  }
  return std::make_unique<Element>(std::move(layout));
}


// Removed make_buffer function

void RamGen::generate_ram_netlist(const int bytes_per_word,
                    const int word_count,
                    const int read_ports,
                    odb::dbMaster* storage_cell,
                    odb::dbMaster* tristate_cell,
                    odb::dbMaster* inv_cell,
                    odb::dbMaster* buf_cell,
                    bool mask) {
    const int bits_per_word = bytes_per_word * 8;
    const std::string ram_name = fmt::format("RAM{}x{}", word_count, bits_per_word);
    logger_->info(RAM, 6, "Generating {}", ram_name);

    storage_cell_ = storage_cell;
    tristate_cell_ = tristate_cell;
    inv_cell_ = inv_cell;
    buf_cell_ = buf_cell;
    and2_cell_ = nullptr;
    clock_gate_cell_ = nullptr;
    findMasters();

    auto chip = db_->getChip();
    if (!chip) chip = odb::dbChip::create(db_);
    block_ = chip->getBlock();
    if (!block_) block_ = odb::dbBlock::create(chip, ram_name.c_str());

    // --- Layout Structure ---
    // Top level is HORIZONTAL: AddrBuf | Decoder | Col0 | Col1 | ...
    auto top_layout = std::make_unique<Layout>(odb::horizontal);

    // --- Create Ports (BTerms) ---
    auto clock = makeBTerm("clock");
    vector<dbNet*> write_enable(bytes_per_word, nullptr);
    for (int byte = 0; byte < bytes_per_word; ++byte) {
        write_enable[byte] = makeBTerm(fmt::format("write_enable[{}]", byte));
    }

    // External Address Inputs
    int address_bits = static_cast<int>(std::ceil(std::log2(word_count)));
    std::vector<dbNet*> address_inputs_external(address_bits);
    for (int i = 0; i < address_bits; ++i) {
        address_inputs_external[i] = makeBTerm(fmt::format("A[{}]", i));
    }
    // Internal Address Nets (after buffer)
    std::vector<dbNet*> address_inputs_internal(address_bits);
    for (int i = 0; i < address_bits; ++i) {
        address_inputs_internal[i] = makeNet("ram", fmt::format("addr[{}]", i));
    }

    // External Data Inputs
    std::vector<dbNet*> data_input_external(bits_per_word);
    for (int bit = 0; bit < bits_per_word; ++bit) {
        data_input_external[bit] = makeBTerm(fmt::format("Din[{}]", bit));
    }
    // Internal Data Inputs (after buffer)
    std::vector<dbNet*> data_input_internal(bits_per_word);
    for (int bit = 0; bit < bits_per_word; ++bit) {
        data_input_internal[bit] = makeNet("ram", fmt::format("Di0[{}]", bit));
    }

    // External Data Outputs
    std::vector<dbNet*> data_output_external(bits_per_word * read_ports);
    int dout_idx_global = 0;
    for (int rp = 0; rp < read_ports; ++rp) {
        for (int bit = 0; bit < bits_per_word; ++bit) {
            data_output_external[dout_idx_global] = makeBTerm(fmt::format("Dout[{}]", dout_idx_global));
            dout_idx_global++;
        }
    }
    // Internal Data Outputs (before buffer) - Create all upfront
    vector<vector<odb::dbNet*>> Do_internal(read_ports, std::vector<odb::dbNet*>(bits_per_word, nullptr));
    for (int rp = 0; rp < read_ports; ++rp) {
        for (int bit = 0; bit < bits_per_word; ++bit) {
             Do_internal[rp][bit] = makeNet("ram", fmt::format("Do_internal_{}_{}", rp, bit));
        }
    }


    // --- Address Buffer ---
    auto address_buffer_layout = std::make_unique<Layout>(odb::vertical, 3.68, 7.675, 0.0, 4.645);  // Increased vertical offset for address buffer
    for (int i = 0; i < address_bits; ++i) {
        makeInst(address_buffer_layout.get(), "address_buffer", fmt::format("buf_{}", i), buf_cell_,
                 {{"A", address_inputs_external[i]}, {"X", address_inputs_internal[i]}});
    }
    odb::Point addr_buf_origin(0, 0);
    address_buffer_layout->position(addr_buf_origin);  // Position will apply offset internally
    top_layout->addElement(std::make_unique<Element>(std::move(address_buffer_layout)));


    // --- Decoder ---
    auto decoder_layout_container = std::make_unique<Layout>(odb::vertical);  // No offset on container
    std::vector<dbNet*> decoder_outputs = buildDecoder(*decoder_layout_container, address_inputs_internal, word_count);
    top_layout->addElement(std::make_unique<Element>(std::move(decoder_layout_container)));


    // --- RAM Columns with Integrated Buffers ---
    for (int col = 0; col < bytes_per_word; ++col) {
        // --- Create the VERTICAL layout for this full column ---
        auto column_layout = std::make_unique<Layout>(odb::vertical, 0.0, 2.805);  // y_offset only

        // Prepare byte-specific internal nets
        array<dbNet*, 8> Di0_byte;
        vector<array<dbNet*, 8>> Do_internal_byte(read_ports);
        for (int bit = 0; bit < 8; ++bit) {
             int global_bit_idx = bit + col * 8;
             Di0_byte[bit] = data_input_internal[global_bit_idx]; // Assign internal input net
             for(int rp=0; rp<read_ports; ++rp) {
                 Do_internal_byte[rp][bit] = Do_internal[rp][global_bit_idx]; // Assign internal output net
             }
        }

        // --- Row Loop for placing buffers and bytes ---
        for (int row = 0; row < word_count; ++row) {
            // --- Place Input Buffers ABOVE the FIRST word ---
            if (row == 0) {
                auto input_buffer_row = std::make_unique<Layout>(odb::horizontal, 
                    11.5, 0.0,  // x_offset only
                    8.28, 0.0);  // x_spacing, y_spacing in microns
                for (int bit = 0; bit < 8; ++bit) {
                    int global_bit_idx = bit + col * 8;
                    makeInst(input_buffer_row.get(),
                             fmt::format("col{}_input_buffer", col),
                             fmt::format("buf_{}", bit),
                             buf_cell_,
                             {{"A", data_input_external[global_bit_idx]},
                              {"X", data_input_internal[global_bit_idx]}});
                }
                odb::Point input_buf_origin(0, 0);
                input_buffer_row->position(input_buf_origin);  // Position will apply offset internally
                column_layout->addElement(std::make_unique<Element>(std::move(input_buffer_row)));
            }

            // --- Place the RAM Byte ---
            auto byte_element = make_byte(fmt::format("storage_{}_{}", row, col),
                            read_ports,
                            clock,
                            write_enable[col],
                            {decoder_outputs[row]},
                            Di0_byte,
                            Do_internal_byte,
                            mask);
            if (!byte_element) return; // Error check
            column_layout->addElement(std::move(byte_element));

            // --- Place Output Buffers BELOW the LAST word ---
            if (row == word_count - 1) {
                auto output_buffer_row = std::make_unique<Layout>(odb::horizontal,
                    11.5, 0.0,  // x_offset only
                    8.28, 0.0);  // x_spacing, y_spacing in microns
                for (int rp = 0; rp < read_ports; ++rp) {
                    for (int bit = 0; bit < 8; ++bit) {
                        int global_bit_idx = bit + col * 8;
                        int external_output_idx = rp * bits_per_word + global_bit_idx;
                        makeInst(output_buffer_row.get(),
                                 fmt::format("col{}_output_buffer_rp{}", col, rp),
                                 fmt::format("buf_{}", bit),
                                 buf_cell_,
                                 {{"A", Do_internal[rp][global_bit_idx]},
                                  {"X", data_output_external[external_output_idx]}});
                    }
                }
                odb::Point output_buf_origin(0, 0);
                output_buffer_row->position(output_buf_origin);  // Position will apply offset internally
                column_layout->addElement(std::make_unique<Element>(std::move(output_buffer_row)));
            }
        } // End row loop

        // Add the completed vertical column layout to the main horizontal top_layout
        top_layout->addElement(std::make_unique<Element>(std::move(column_layout)));
    } // End col loop


    // --- Final Layout Positioning ---
    top_layout->position(odb::Point(0, 0));
}

// buildDecoder, findMasters, isAndGate, isAndGateFunction, getAndGate, findMaster remain the same...
std::vector<dbNet*> RamGen::buildDecoder(Layout& parent_layout, const std::vector<dbNet*>& address_nets, int word_count)
{
  int address_bits = static_cast<int>(address_nets.size());
  auto decoder_layout_internal = std::make_unique<Layout>(odb::horizontal); // Internal layout for decoder logic
  auto inv_layout = std::make_unique<Layout>(odb::vertical, 0.0, 7.675, 0.0, 4.645);  // Increased vertical offset for inverter layout

  if (word_count == 4 && address_bits == 2) {
    auto and_layout = std::make_unique<Layout>(odb::vertical, 0.0, 5.525);  // Keep AND layout at original offset
    auto A0 = address_nets[0];
    auto A1 = address_nets[1];
    auto A0_b = makeNet("decoder", "A0_b");
    auto A1_b = makeNet("decoder", "A1_b");
    makeInst(inv_layout.get(), "decoder", "invA0", inv_cell_, {{"A", A0}, {"Y", A0_b}});
    makeInst(inv_layout.get(), "decoder", "invA1", inv_cell_, {{"A", A1}, {"Y", A1_b}});
    std::vector<dbNet*> outputs(4);
    for(int i=0; i<4; ++i) outputs[i] = makeNet("decoder", fmt::format("word_select[{}]", i));
    makeInst(and_layout.get(), "decoder", "and0", and2_cell_, {{"A", A1_b}, {"B", A0_b}, {"X", outputs[0]}});
    makeInst(and_layout.get(), "decoder", "and1", and2_cell_, {{"A", A1_b}, {"B", A0},   {"X", outputs[1]}});
    makeInst(and_layout.get(), "decoder", "and2", and2_cell_, {{"A", A1},   {"B", A0_b}, {"X", outputs[2]}});
    makeInst(and_layout.get(), "decoder", "and3", and2_cell_, {{"A", A1},   {"B", A0},   {"X", outputs[3]}});
    decoder_layout_internal->addElement(std::make_unique<Element>(std::move(inv_layout)));
    decoder_layout_internal->addElement(std::make_unique<Element>(std::move(and_layout)));
    parent_layout.addElement(std::make_unique<Element>(std::move(decoder_layout_internal)));
    return outputs;
  } else if (word_count == 8 && address_bits == 3) {
    auto and_logic_layout = std::make_unique<Layout>(odb::horizontal); // Layout for the AND gate logic part
    auto A0 = address_nets[0];
    auto A1 = address_nets[1];
    auto A2 = address_nets[2];
    auto A0_b = makeNet("decoder", "A0_b");
    auto A1_b = makeNet("decoder", "A1_b");
    auto A2_b = makeNet("decoder", "A2_b");
    makeInst(inv_layout.get(), "decoder", "invA0", inv_cell_, {{"A", A0}, {"Y", A0_b}});
    makeInst(inv_layout.get(), "decoder", "invA1", inv_cell_, {{"A", A1}, {"Y", A1_b}});
    makeInst(inv_layout.get(), "decoder", "invA2", inv_cell_, {{"A", A2}, {"Y", A2_b}});
    std::vector<dbNet*> outputs(8);
    for (int i = 0; i < 8; i++) outputs[i] = makeNet("decoder", fmt::format("word_select[{}]", i));
    auto triple_and = [&](Layout* layout_ptr, const std::string& prefix, dbNet* in1, dbNet* in2, dbNet* in3, dbNet* out) {
        auto mid_net = makeNet("decoder", fmt::format("{}_mid", prefix));
        makeInst(layout_ptr, "decoder", fmt::format("{}_and1", prefix), and2_cell_, {{"A", in1}, {"B", in2}, {"X", mid_net}});
        makeInst(layout_ptr, "decoder", fmt::format("{}_and2", prefix), and2_cell_, {{"A", mid_net}, {"B", in3}, {"X", out}});
    };
    auto and_layout_top = std::make_unique<Layout>(odb::vertical, 0.0, 5.525);  // Keep AND layouts at original offset
    auto and_layout_bottom = std::make_unique<Layout>(odb::vertical, 0.0, 5.525);  // Keep AND layouts at original offset
    triple_and(and_layout_top.get(), "out0", A2_b, A1_b, A0_b, outputs[0]);
    triple_and(and_layout_top.get(), "out1", A2_b, A1_b, A0,   outputs[1]);
    triple_and(and_layout_top.get(), "out2", A2_b, A1,   A0_b, outputs[2]);
    triple_and(and_layout_top.get(), "out3", A2_b, A1,   A0,   outputs[3]);
    triple_and(and_layout_bottom.get(), "out4", A2, A1_b, A0_b, outputs[4]);
    triple_and(and_layout_bottom.get(), "out5", A2, A1_b, A0,   outputs[5]);
    triple_and(and_layout_bottom.get(), "out6", A2, A1,   A0_b, outputs[6]);
    triple_and(and_layout_bottom.get(), "out7", A2, A1,   A0,   outputs[7]);
    and_logic_layout->addElement(std::make_unique<Element>(std::move(and_layout_top)));
    and_logic_layout->addElement(std::make_unique<Element>(std::move(and_layout_bottom)));
    decoder_layout_internal->addElement(std::make_unique<Element>(std::move(inv_layout))); // Add inverters
    decoder_layout_internal->addElement(std::make_unique<Element>(std::move(and_logic_layout))); // Add AND logic
    parent_layout.addElement(std::make_unique<Element>(std::move(decoder_layout_internal)));
    return outputs;
  } else {
    logger_->error(RAM, 12, "Unsupported decoder configuration (word_count = {}). Only 4 or 8 words are supported.", word_count);
    return {};
  }
}

void RamGen::findMasters() {
  if (!and2_cell_) {
    and2_cell_ = findMaster( [this](sta::LibertyPort* port) { return isAndGate(port, 2); }, "and2");
  }
  max_and_inputs_ = 2;
  if(and2_cell_) and_cells_[2] = and2_cell_;

  if (!clock_gate_cell_) {
    clock_gate_cell_ = findMaster( [this](sta::LibertyPort* port) { return port->libertyCell()->isClockGate(); }, "clock gate");
  }
}

bool RamGen::isAndGate(sta::LibertyPort* port, int num_inputs) {
  if (!port || !port->direction() || !port->direction()->isOutput()) return false;
  auto function = port->function();
  if (!function) return false;
  int inputs_count = 0;
  bool is_and_gate = isAndGateFunction(function, inputs_count);
  return is_and_gate && inputs_count == num_inputs;
}

bool RamGen::isAndGateFunction(sta::FuncExpr* expr, int& inputs_count) {
  if (!expr) return false;
  if (expr->op() == sta::FuncExpr::op_port) {
    inputs_count += 1;
    return true;
  } else if (expr->op() == sta::FuncExpr::op_and) {
    return isAndGateFunction(expr->left(), inputs_count) && isAndGateFunction(expr->right(), inputs_count);
  } else {
    return false;
  }
}


odb::dbMaster* RamGen::getAndGate(int num_inputs) {
  if (num_inputs != 2) {
      logger_->error(RAM, 105, "Requested AND gate with {} inputs, but only 2-input is supported.", num_inputs);
      return nullptr;
  }
  return and2_cell_;
}

odb::dbMaster* RamGen::findMaster(
    const std::function<bool(sta::LibertyPort*)>& match,
    const char* name) {
  dbMaster* best = nullptr;
  float best_area = std::numeric_limits<float>::max();
  for (auto lib : db_->getLibs()) {
    for (auto master : lib->getMasters()) {
      auto cell = network_->dbToSta(master);
      if (!cell) continue;
      auto liberty = network_->libertyCell(cell);
      if (!liberty) continue;
      auto port_iter = liberty->portIterator();
      sta::ConcretePort* out = nullptr;
      while (port_iter->hasNext()) {
        auto lib_port = port_iter->next();
         if (!lib_port) continue;
        auto dir = lib_port->direction();
        if (dir && dir->isAnyOutput()) {
          if (!out) out = lib_port;
          else { out = nullptr; break; }
        }
      }
      delete port_iter;
       if (!out || !out->libertyPort() || !match(out->libertyPort())) continue;
      if (liberty->area() < best_area) {
        best_area = liberty->area();
        best = master;
      }
    }
  }
  if (!best) {
    logger_->error(RAM, 10, "Can't find {} cell", name);
  } else {
    logger_->info(RAM, 16, "Selected {} cell {}", name, best->getName());
  }
  return best;
}


} // namespace ram
