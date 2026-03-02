# 🤖 Ramię Robota Sterowane Wizyjnie (Python + LabVIEW)

[![Python](https://img.shields.io/badge/Python-3.x-blue.svg)](https://www.python.org/)
[![LabVIEW](https://img.shields.io/badge/LabVIEW-Project-yellow.svg)](https://www.ni.com/pl-pl/shop/labview.html)
[![Hardware](https://img.shields.io/badge/Hardware-PCA_Board-green.svg)]()

Ten projekt to w pełni funkcjonalne ramię robota, którym można sterować za pomocą gestów/ruchu przechwytywanego przez kamerę w czasie rzeczywistym. System stanowi unikalne połączenie wysokopoziomowej analizy obrazu oraz autorskiego sterowania sprzętem.

## ✨ Główne cechy projektu

* **👀 Sterowanie wizyjne (Python):** Za przechwytywanie i analizę obrazu z kamery odpowiada skrypt napisany w Pythonie. Śledzi on ruch i wysyła odpowiednie komendy sterujące.
* **⚙️ Autorskie oprogramowanie (LabVIEW):** Główny silnik logiczny projektu został napisany w środowisku LabVIEW.
* **🛠️ Własne sterowniki płytki PCA:** Zamiast korzystać z gotowych bibliotek, sterowniki do obsługi kontrolera serwomechanizmów (płytka PCA) napisałem **od zera samodzielnie w LabVIEW**, co zapewnia pełną kontrolę nad sprzętem i niskie opóźnienia.
* **🔗 Płynna komunikacja:** Zoptymalizowana wymiana danych pomiędzy skryptem wizyjnym w Pythonie a programem w LabVIEW.

---

## 📸 Historia budowy (Galeria)

Projekt ewoluował od luźnych części aż po w pełni zmontowane ramię. Poniżej znajduje się krótka historia wizualna z procesu konstrukcji:

<div align="center">
  <img src="[TUTAJ_WSTAW_LINK_DO_ZDJĘCIA_1]" width="30%" alt="Początki budowy">
  <img src="[TUTAJ_WSTAW_LINK_DO_ZDJĘCIA_2]" width="30%" alt="Składanie elektroniki">
  <img src="[TUTAJ_WSTAW_LINK_DO_ZDJĘCIA_3]" width="30%" alt="Gotowe ramię">
</div>

*1. Pierwsze przymiarki elementów mechanicznych. | 2. Podłączanie płytki PCA i serwomechanizmów. | 3. Ramię gotowe do testów wizyjnych.*

---

## 🛠️ Architektura Systemu

1. **Kamera** rejestruje obraz.
2. **Skrypt Python** (moduł wizyjny) analizuje klatki, wykrywa np. pozycję dłoni/markera i tłumaczy je na współrzędne w przestrzeni.
3. Współrzędne przesyłane są do oprogramowania **LabVIEW**.
4. **Program LabVIEW** przelicza otrzymane dane na kąty wychylenia poszczególnych przegubów ramienia (kinematyka).
5. Autorski sterownik (LabVIEW) wysyła sygnały sterujące do **płytki PCA**, która generuje sygnał PWM dla serwomechanizmów.

---

## 🚀 Jak uruchomić projekt?

### Wymagania
* Zainstalowane środowisko **LabVIEW** (wersja [WPISZ WERSJĘ, np. 2020 lub nowsza]).
* Zasilacz do serwomechanizmów oraz podłączona płytka PCA.
* **Python 3.x** oraz potrzebne biblioteki. Aby je zainstalować, uruchom w terminalu:
  ```bash
  pip install -r requirements.txt
