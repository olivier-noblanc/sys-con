#include "Controllers/ShanWanPantherLordController.h"

namespace
{
    // Slots "raw" utilises par rawData->buttons[] - correspondent a la section
    // config.ini [0810-0001] (A=1, B=2, X=3, Y=4, L=5, R=6, ZL=7, ZR=8,
    // minus=9, plus=10, lstick_click=11, rstick_click=12).
    enum RawButtonSlot
    {
        RAW_A = 1,
        RAW_B = 2,
        RAW_X = 3,
        RAW_Y = 4,
        RAW_L1 = 5,
        RAW_R1 = 6,
        RAW_L2 = 7,
        RAW_R2 = 8,
        RAW_SELECT = 9,
        RAW_START = 10,
        RAW_L3 = 11,
        RAW_R3 = 12,
    };

    float NormalizeAxis(uint8_t raw, bool invert)
    {
        float v = (static_cast<float>(raw) - 128.0f) / 128.0f;
        if (v > 1.0f)
            v = 1.0f;
        if (v < -1.0f)
            v = -1.0f;
        return invert ? -v : v;
    }
} // namespace

ShanWanPantherLordController::ShanWanPantherLordController(
    std::unique_ptr<IUSBDevice> &&device,
    const ControllerConfig &config,
    std::unique_ptr<ILogger> &&logger)
    : BaseController(std::move(device), config, std::move(logger))
{
    m_logger->Log(LogLevelInfo, "ShanWanPantherLord[%04x-%04x] Created !",
                  m_device->GetVendor(), m_device->GetProduct());
}

ShanWanPantherLordController::~ShanWanPantherLordController()
{
}

uint16_t ShanWanPantherLordController::GetInputCount()
{
    return SHANWAN_INPUT_COUNT;
}

ControllerResult ShanWanPantherLordController::ReadNextBuffer(uint8_t *buffer, size_t *size, uint16_t *input_idx, uint32_t timeout_us)
{
    // Pas de "drain and keep latest" ici : ce device multiplexe 2 manettes
    // (report ID en buffer[0]) sur le meme endpoint. Le comportement par defaut
    // de BaseController jetterait systematiquement l'un des deux IDs a chaque
    // cycle. On lit donc un seul paquet a la fois, sans drainer.
    if (m_inPipe.empty())
        return CONTROLLER_STATUS_NOTHING_TODO;

    ControllerResult result = m_inPipe[0]->Read(buffer, size, timeout_us);
    if (result != CONTROLLER_STATUS_SUCCESS)
        return result;

    if (*size == 0)
        return CONTROLLER_STATUS_NOTHING_TODO;

    if (input_idx != NULL)
        *input_idx = 0; // sera ecrase par ParseData en fonction de buffer[0]

    return CONTROLLER_STATUS_SUCCESS;
}

ControllerResult ShanWanPantherLordController::ParseData(uint8_t *buffer, size_t size, RawInputData *rawData, uint16_t *input_idx)
{
    if (size < SHANWAN_REPORT_SIZE)
    {
        m_logger->Log(LogLevelError, "ShanWanPantherLord[%04x-%04x] Unexpected report size: %d (expected >= %d)",
                      m_device->GetVendor(), m_device->GetProduct(), (int)size, SHANWAN_REPORT_SIZE);
        return CONTROLLER_STATUS_UNEXPECTED_DATA;
    }

    uint8_t report_id = buffer[0]; // 0x01 = port 1, 0x02 = port 2
    if (input_idx != NULL)
        *input_idx = (report_id == 0x02) ? 1 : 0;

    uint8_t rstick_x = buffer[1];
    uint8_t rstick_y = buffer[2];
    uint8_t lstick_x = buffer[3];
    uint8_t lstick_y = buffer[4];
    uint8_t face_hat = buffer[5];
    uint8_t shoulders = buffer[6];

    // Sticks (convention du projet : X/Y = stick gauche, Z/Rz = stick droit)
    rawData->analog[ControllerAnalogType_X] = NormalizeAxis(lstick_x, false); // droite = +1
    rawData->analog[ControllerAnalogType_Y] = NormalizeAxis(lstick_y, true);  // haut = +1
    rawData->analog[ControllerAnalogType_Z] = NormalizeAxis(rstick_x, false);
    rawData->analog[ControllerAnalogType_Rz] = NormalizeAxis(rstick_y, true);

    // Boutons face (nibble haut de byte5)
    rawData->buttons[RAW_A] = (face_hat & 0x40) != 0;
    rawData->buttons[RAW_B] = (face_hat & 0x20) != 0;
    rawData->buttons[RAW_X] = (face_hat & 0x80) != 0;
    rawData->buttons[RAW_Y] = (face_hat & 0x10) != 0;

    // Dpad (nibble bas de byte5, encodage hat-switch : 0=Haut 2=Droite 4=Bas 6=Gauche, repos=0xF)
    uint8_t hat = face_hat & 0x0F;
    bool up = false, down = false, left = false, right = false;
    switch (hat)
    {
    case 0: up = true; break;
    case 1: up = true; right = true; break;
    case 2: right = true; break;
    case 3: right = true; down = true; break;
    case 4: down = true; break;
    case 5: down = true; left = true; break;
    case 6: left = true; break;
    case 7: left = true; up = true; break;
    default: break; // neutre
    }
    rawData->buttons[DPAD_UP_BUTTON_ID] = up;
    rawData->buttons[DPAD_DOWN_BUTTON_ID] = down;
    rawData->buttons[DPAD_LEFT_BUTTON_ID] = left;
    rawData->buttons[DPAD_RIGHT_BUTTON_ID] = right;

    // Boutons epaules/systeme (bitmask byte6)
    rawData->buttons[RAW_L1] = (shoulders & 0x01) != 0;
    rawData->buttons[RAW_R1] = (shoulders & 0x02) != 0;
    rawData->buttons[RAW_L2] = (shoulders & 0x04) != 0;
    rawData->buttons[RAW_R2] = (shoulders & 0x08) != 0;
    rawData->buttons[RAW_SELECT] = (shoulders & 0x10) != 0;
    rawData->buttons[RAW_START] = (shoulders & 0x20) != 0;
    rawData->buttons[RAW_L3] = (shoulders & 0x40) != 0;
    rawData->buttons[RAW_R3] = (shoulders & 0x80) != 0;

    return CONTROLLER_STATUS_SUCCESS;
}