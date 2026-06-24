# config/

Аппаратные конфиги драйвера youBot:
- `youbot-base.cfg`, `youbot-manipulator.cfg` — параметры базы и манипулятора;
- `youbot-ethercat.cfg` — EtherCAT (см. `docs/ETHERCAT_TROUBLESHOOTING.md`).

## Главный файл параметров задачи

Единый файл всех параметров задачи (точки, цвет кубика, препятствия, seed,
скорости и т.д.) находится в пакете perception:

    src/youbot_perception/config/mission_params.yaml

Меняйте значения ТОЛЬКО там — они подхватываются всеми нодами через
namespace `/mission_config`.
