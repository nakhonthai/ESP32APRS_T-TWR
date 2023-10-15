#include "igate.h"

extern WiFiClient aprsClient;
extern Configuration config;

int igateProcess(AX25Msg &Packet)
{
    int idx, j;
    String header;

    j = 0;
    if (Packet.len < 2)
    {
        // digiLog.ErPkts++;
        return 0; // NO INFO DATA
    }

    for (idx = 0; idx < Packet.rpt_count; idx++)
    {
        if (!strncmp(&Packet.rpt_list[idx].call[0], "RFONLY", 6))
        {
            // digiLog.DropRx++;
            return 0;
        }
    }

    for (idx = 0; idx < Packet.rpt_count; idx++)
    {
        if (!strncmp(&Packet.rpt_list[idx].call[0], "TCPIP", 5))
        {
            // digiLog.DropRx++;
            return 0;
        }
    }

    for (idx = 0; idx < Packet.rpt_count; idx++)
    {
        if (!strncmp(&Packet.rpt_list[idx].call[0], "qA", 2))
        {
            // digiLog.DropRx++;
            return 0;
        }
    }

    header = String(Packet.src.call);
    if (Packet.src.ssid > 0)
    {
        header += String(F("-"));
        header += String(Packet.src.ssid);
    }
    header += String(F(">"));
    header += String(Packet.dst.call);
    if (Packet.dst.ssid > 0)
    {
        header += String(F("-"));
        header += String(Packet.dst.ssid);
    }

    // Add Path
    for (int i = 0; i < Packet.rpt_count; i++)
    {
        header += String(",");
        header += String(Packet.rpt_list[i].call);
        if (Packet.rpt_list[i].ssid > 0)
        {
            header += String("-");
            header += String(Packet.rpt_list[i].ssid);
        }
        if (Packet.rpt_flags & (1 << i))
            header += "*";
    }

    if (strlen((const char *)config.igate_object) >= 3)
    {
        header += "," + String(config.aprs_mycall);
        if (config.aprs_ssid > 0)
        {
            header += String(F("-"));
            header += String(config.aprs_ssid);
        }
        header += "*,qAO," + String(config.igate_object);
    }
    else
    {
        // Add qAR,callSSID: qAR - Packet is placed on APRS-IS by an IGate from RF
        header += String(F(",qAR,"));
        header += String(config.aprs_mycall);
        if (config.aprs_ssid > 0)
        {
            header += String(F("-"));
            header += String(config.aprs_ssid);
        }
    }

    // Add Infomation
    header += String(F(":"));
    uint8_t Raw[500];
    memset(Raw, 0, sizeof(Raw)); // Clear frame packet
    size_t hSize = strlen(header.c_str());
    memcpy(&Raw[0], header.c_str(), hSize);           // Copy header to frame packet
    memcpy(&Raw[hSize], &Packet.info[0], Packet.len); // Copy info to frame packet
    aprsClient.write(&Raw[0], hSize + Packet.len);    // Send binary frame packet to APRS-IS (aprsc)
    aprsClient.println();                             // Send CR LF the end frame packet
    log_d("RF2INET: %s", Raw);
    return 1;
}