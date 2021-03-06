# VDC
...Проект в разработке..

main - VDC.ino

Учебный проект «Ветеринарный диагностический комплекс» был инициирован
ФГАОУВО «Московский политех» совместно с ветеринарной клиникой
«Умка» в связи с высокой актуальностью и практической значимостью
данной работы для повседневной работы врача-ветеринара.

В настоящее время аналогов подобных устройств не существует,
несмотря на их высокую востребованность для диагностики и мониторинга
состояния животных.
   
Проект находится на стадии разработки.

1. Концепция проекта

1.1. Основные положения
Предметом разработки являются комплекс, осуществляющий замер и передачу жизненных показателей питомца, и мобильное приложение с функционалом, позволяющим пользователям ознакомится с переданными показателями, следить за их динамикой.  

1.2. Пользователи системы
Пользователи системы – владельцы домашних питомцев (кошек и собак), ветеринарные клиники.

1.3. Функционал
Устройство должно выполнять следующие функции:
•	замер пульса, температуры, мышечной активности и частоты дыхания питомца;
•	производить обработку и передачу данных в облачное хранилище;
•	показывать местоположение питомца;
•	если передача не возможна, то данные должны поместится во внешнее хранилище, а при подключении к сети, записаться в БД;
У пользователей мобильного приложения должны быть следующие возможности:
●	регистрация пары устройство + пользователь в БД;
●	создание профиля своего питомца;
●	просмотр текущих показателей с устройства;
●	просмотр статистики;
●	общение с ветеринаром через чат;
●	делать заметки по календарю.

1.4. Хранение данных
В базе данных серверной части должны храниться следующие данные:
●	Данные о пользовательских аккаунтах: Имя пользователя, контактные данные пользователя (телефон, адрес), характеристики питомца (кличка, порода, возраст, рост в холке, вес, описание питомца (текст 100-200 символов максимум)), ФИО привязанного ветеринара, номер клиники (можно сделать просто цифрами), id связанного устройства. 
●	Данные о переданных показателях с датами передачи: пульс, температура, мышечная активность, частота дыхания. ОБЯЗАТЕЛЬНО продумать, как хранить интервалы измерений – пользователь задает их на телефоне и передает на сервер (по идее, это таблица из двух столбиков - измеряемая величина/интервал в секундах).
Задачи +
•	Заметки пользователя по датам (как календарь или заметки на телефоне)
•	Серверная часть для чата с ветеринаром.
•	Таблица с ветеринарами (ФИО, номер клиники, контактный телефон, должность) 


2. Требования
Устройство должно:
•	быть автономным (работать без подключения к сети);
•	передавать данные в сеть на основе технологии - Wi-Fi;
•	быть безопасно и защищено от внешних воздействий;
•	осуществлять связь со смартфоном с помощью bluetooth;
•	иметь объем внешней flash памяти - 256 мб ;
•	иметь свой личный id;
•	производить измерения с датчиков с указанным в приложении интервалом;
•	включать в себя датчики: температуры, частоты дыхания, мышечной активности, частоты пульса;
•	передавать данные о месте положения питомца(GPS)
процессор:
Приложение:

•	Регистрация:
1.	Регистрация по телефону производится единожды.
2.	Идет запись в таблицу БД (id, мобильный номер. id устройства).
3.	Подключение устройства к аккаунту происходит через bluetooth в приложении.
4.	Окно поиска устройства должно показывать сторонние девайсы. 
5.	Анкетирование пользователя включает: имя, имя ветеринара (и его контакты), характеристики питомца (кличка, порода, возраст и т.д.)

•	Интерфейс главного экрана должен: 
1.	Быть визуально красив и приятен.
2.	Отображать текущие показатели питомца.
3.	Давать возможность выбрать интервал измерения.
4.	Включать чат с ветеринаром.
5.	Отображать заряд главного устройства.

•	Интерфейс экрана статистики должен:
1.	Отображать учет показателей за месяц.
2.	Показывать графики с выбранным интервалом.
3.	Должен давать возможность отображать выбранные показатели

•	Интерфейс экрана учетной записи должен:
1.	Отображать данные о пользователе и питомце.
2.	Давать возможность изменить данные.
3.	Давать возможность сбросить привязанное устройство.

•	Интерфейс экрана заметок должен:
1.	Предоставлять возможность поставить заметку на конкретный день или период и изменить ее. Это может быть нестандартное поведение питомца, количество принятых препаратов и т.д.

