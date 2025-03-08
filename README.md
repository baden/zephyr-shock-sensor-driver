# Акустичний датчик удару

Акустичний датчик удару для Zephyr RTOS.

## Supported Zephyr versions
* 4.1.0 (має працювати і в попередніх версіях)

## Usage
### Module installation

Додай в свій `west.yml` маніфест проєкту:

```yaml
- name: shock-sensor-driver
  path: modules/shock-sensor-driver
  url: https://github.com/baden/zephyr-shock-sensor-driver.git
  revision: main
```

Тож ваші проєкти повинні виглядати приблизно так:

```yaml
manifest:
  projects:
    - name: zephyr
      url: https://github.com/zephyrproject-rtos/zephyr
      revision: refs/tags/zephyr-v4.1.0
      import: true
    - name: sensirion_drivers
      path: modules/sensirion_drivers
      url: https://github.com/nobodyguy/sensirion_zephyr_drivers
      revision: main
```

Це імпортує модуль і дозволить вам використовувати драйвери у вашому коді.

Крім того, переконайтеся, що ви запустили west update, коли додали цей запис до свого west.yml.

### Використання драйвера

Будь ласка, перевірте приклади в каталозі ./samples/ (TODO).
