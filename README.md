# Systemy rozproszone - Blockchain

## Założenia:
System zostanie oparty o framework [ROS](http://wiki.ros.org/), który pozwala na komunikację P2P pomiędzy klientami zarejestrowanymi na serwerze master znajdującym się lokalnie na jedym z komputerów.

## Sposób działania systemu:
* __Proces/komputer__ - Instancja aplikacji ROS - "Node" odpowiedzialny za obsługę sieci blockchain. Bierze udział w głosowaniach i informuje klientów o stanie transakcji.

* __Klient__ - Użytkownik sieci blockchain, który ma możliwość komunikacji z innymi użytkownikami, posiada własny portfel i może inicjalizować transakcje.

* __Głosowanie__ - Każda transakcja musi zostać zatwierdzona przez conajmniej 51% obecnych użytkowników sieci. W momencie inicjalizacji przelewu przez klienta wybiernay jest losowy komputer, który będzie odpowiedzialny za przeprowadzenie głosowania i potwierdzenie/odrzucenie prośby transakcji.

* __Zabezpieczenie transakcji__ - enkrypcja RSA?

## Przypadki użycia:

* Zdecentralizowane sieci wymiany danych.
* Zdecentralizowane systemy transakcji (kryptowaluty).
* Nadzór przelewów bankowych.