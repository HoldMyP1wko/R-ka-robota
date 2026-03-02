import pandas as pd
import matplotlib.pyplot as plt

data = [120, 135, 142, 128, 150, 160, 145, 138, 132, 140, 155, 148, 149, 151, 152,
        300, 310, 295, 305, 290, 148, 147, 149, 150, 151, 152, 153, 140, 142, 141, 
        139, 138, 137, 136, 135, 134, 133, 132, 131, 130]
df = pd.DataFrame(data, columns=['response_time_ms'])

# Obliczenia statystyczne
stats = {
    "Średnia": df['response_time_ms'].mean(),
    "Wariancja": df['response_time_ms'].var(),
    "Odchylenie standardowe": df['response_time_ms'].std(),
    "Mediana": df['response_time_ms'].median(),
    "Moda": df['response_time_ms'].mode().tolist()
}

print("Wyniki:", stats)

# Generowanie wykresów
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
ax1.hist(df['response_time_ms'], bins=15, color='skyblue', edgecolor='black')
ax1.set_title('Histogram czasu odpowiedzi')
ax2.boxplot(df['response_time_ms'], vert=False, patch_artist=True)
ax2.set_title('Wykres pudełkowy')
plt.show()