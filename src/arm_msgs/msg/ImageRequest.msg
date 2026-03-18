uint8 image_id
bool compress
uint8 quality          # SSDV quality 0-7 (0=lowest, 7=highest)
bool fec               # Use Forward Error Correction (Reed-Solomon)
uint16 max_packet_size # SSDV packet size in bytes (max 256, default 256)
