#pragma once

#include "Controllers/BaseController.h"
#include <memory>

// ShanWan / PantherLord 2.4GHz Dual USB adapter - VID 0x0810, PID 0x0001
//
// Descripteur HID minimal non exploitable par le parser generique (HIDJoystick) :
// mapping determine empiriquement par capture USB directe. Report brut 8 octets,
// MULTIPLEXE sur le meme endpoint IN via un ID de manette en byte[0] :
//   byte0 : ID manette (0x01 = port 1, 0x02 = port 2)
//   byte1 : stick droit X   (0x00 gauche .. 0xFF droite, centre 0x80)
//   byte2 : stick droit Y   (0x00 haut   .. 0xFF bas,    centre 0x80)
//   byte3 : stick gauche X  (0x00 gauche .. 0xFF droite, centre 0x80)
//   byte4 : stick gauche Y  (0x00 haut   .. 0xFF bas,    centre 0x80)
//   byte5 : nibble haut = boutons face (bit4=Y bit5=B bit6=A bit7=X)
//           nibble bas  = dpad facon hat-switch (0=Haut 2=Droite 4=Bas 6=Gauche, F=repos)
//   byte6 : bitmask boutons epaules/systeme
//           bit0=L1 bit1=R1 bit2=L2 bit3=R2 bit4=Select bit5=Start bit6=L3 bit7=R3
//   byte7 : inutilise
//
// IMPORTANT : le dongle envoie les 2 reports (0x01 et 0x02) en continu, colles a
// ~15ms d'ecart, meme si un seul port est physiquement occupe. Le comportement par
// defaut de BaseController::ReadEndpointLatest ("drain and keep only latest") jette
// systematiquement l'un des deux a chaque cycle -> ReadNextBuffer est donc surcharge
// ici pour lire un seul paquet a la fois, sans drainer.

#define SHANWAN_VID 0x0810
#define SHANWAN_PID 0x0001
#define SHANWAN_REPORT_SIZE 8
#define SHANWAN_INPUT_COUNT 2

class ShanWanPantherLordController : public BaseController
{
public:
    ShanWanPantherLordController(std::unique_ptr<IUSBDevice> &&device, const ControllerConfig &config, std::unique_ptr<ILogger> &&logger);
    ~ShanWanPantherLordController() override;

    uint16_t GetInputCount() override;

protected:
    ControllerResult ReadNextBuffer(uint8_t *buffer, size_t *size, uint16_t *input_idx, uint32_t timeout_us) override;
    ControllerResult ParseData(uint8_t *buffer, size_t size, RawInputData *rawData, uint16_t *input_idx) override;
};