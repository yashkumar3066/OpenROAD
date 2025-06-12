#############################################################################
## BSD 3-Clause License ... (rest of license header) ...
#############################################################################

sta::define_cmd_args "generate_ram_netlist" {-bytes_per_word bytes
                                             -word_count words
                                             -storage_cell name
                                             -tristate_cell name
                                             -inv_cell name
                                             -buf_cell name  
                                             -read_ports count
                                             [-mask]}

proc generate_ram_netlist { args } {
  sta::parse_key_args "generate_ram_netlist" args \
      keys {-bytes_per_word -word_count -storage_cell -tristate_cell -inv_cell \
            -buf_cell -read_ports} flags {-mask} ;# Added -buf_cell

  if { ![info exists keys(-bytes_per_word)] } { utl::error RAM 1 "The -bytes_per_word argument must be specified." }
  set bytes_per_word $keys(-bytes_per_word)

  if { ![info exists keys(-word_count)] } { utl::error RAM 2 "The -word_count argument must be specified." }
  set word_count $keys(-word_count)

  if {$word_count != 4 && $word_count != 8} { utl::error RAM 22 "Only 4 or 8 words are supported."; return }

  if { ![info exists keys(-storage_cell)] } { utl::error RAM 3 "The -storage_cell argument must be specified." }
  set storage_cell $keys(-storage_cell)

  if { ![info exists keys(-tristate_cell)] } { utl::error RAM 4 "The -tristate_cell argument must be specified." }
  set tristate_cell $keys(-tristate_cell)

  if { ![info exists keys(-inv_cell)] } { utl::error RAM 5 "The -inv_cell argument must be specified." }
  set inv_cell $keys(-inv_cell)

  # Check for buffer cell argument
  if { ![info exists keys(-buf_cell)] } { utl::error RAM 107 "The -buf_cell argument must be specified." } ;# New check
  set buf_cell $keys(-buf_cell)

  set read_ports 1
  if { [info exists keys(-read_ports)] } { set read_ports $keys(-read_ports) }

  set mask [expr {[info exists flags(-mask)] ? 1 : 0}]

  # Call C++ command with the new buf_cell argument
  ram::generate_ram_netlist_cmd $bytes_per_word $word_count $storage_cell \
      $tristate_cell $inv_cell $buf_cell $read_ports $mask
}
