# The header
Header header

# The Status Profile part of the message
# Refer UDP specification pg 8 Table 3.3
uint8 image_type
uint8 op_mode
uint8 error_bits
uint8 detection_state_bits
uint8 triple_sel
uint8 aux
uint8 input_bits
uint8 output_bits
uint16 volt
int16 temp
int16 level_x
int16 level_y
uint32 scan_number
uint32 safe_sig
uint32 err_data


#The Measurement Contour Description part of the message
# Refer UDP specification pg 13 Table 3.4
uint16 start_index
uint16 stop_index
uint16 index_interval
uint16 reserved
