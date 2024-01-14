import sys
import pygame
import random
import numpy as np

class Cercle:
    def __init__(self, screen, x, y, radius, mass, velocity):
        self.screen = screen
        self.x = x
        self.y = y
        self.radius = radius
        self.mass = mass
        self.velocity = np.array(velocity, dtype=np.float64)

    def draw(self):
        pygame.draw.circle(self.screen, (255, 255, 255), (int(self.x), int(self.y)), self.radius)

def calculer_force_gravitationnelle(m1, m2, distance):

    force = G * (m1 * m2) / (distance**2)
    return force

def calculer_vecteur_force(etoile1, etoile2):
    dx = etoile2.x - etoile1.x
    dy = etoile2.y - etoile1.y
    distance = np.sqrt(dx**2 + dy**2)

    force = calculer_force_gravitationnelle(etoile1.mass, etoile2.mass, distance)
    angle = np.arctan2(dy, dx)

    force_x = force * np.cos(angle)
    force_y = force * np.sin(angle)

    return force_x, force_y


def collision_rebond(circle1, circle2, normal):
    # Calculer les vitesses relatives avant la collision dans le repère normal
    relative_velocity = circle2.velocity - circle1.velocity
    v_relative_normal = np.dot(relative_velocity, normal)

    # Vérifier si les cercles se dirigent l'un vers l'autre (éviter la fusion)
    if v_relative_normal > 0:
        return

    # Calculer les masses réduites
    m1 = circle1.mass
    m2 = circle2.mass
    m_r = (2 * m2) / (m1 + m2)

    # Calculer les vitesses après la collision dans le repère normal (conservation de l'énergie cinétique)
    v1_normal_after = ((m1 - m2) * v_relative_normal + 2 * m2 * np.dot(circle1.velocity, normal)) / (m1 + m2)
    v2_normal_after = ((m2 - m1) * v_relative_normal + 2 * m1 * np.dot(circle2.velocity, normal)) / (m1 + m2)

    # Calculer les vecteurs de vitesse après la collision
    circle1.velocity += (v1_normal_after - v_relative_normal) * normal
    circle2.velocity += (v2_normal_after - v_relative_normal) * normal

pygame.init()
screen_info = pygame.display.Info()
width, height = screen_info.current_w - 500, screen_info.current_h - 500
screen = pygame.display.set_mode((width, height), pygame.RESIZABLE)
cercle_num = random.randint(2, 10)

# Distance depuis le centre
distances_from_center = np.random.uniform(200, 400, cercle_num)
center_x, center_y = width // 2, height // 2

# Liste pour stocker les objets cercle
circles = []

circles.append(Cercle(screen, center_x , center_y, 50, 1000, [0, 0]))
G = 6.67430e-11  # Constante gravitationnelle
# Créer les cercles avec NumPy
angles = np.linspace(0, 2 * np.pi, cercle_num, endpoint=False)
circles_coordinates = np.array([
    (center_x + distance * np.cos(angle),
     center_y + distance * np.sin(angle))
    for distance, angle in zip(distances_from_center, angles)
])
distance_to_center_x = distances_from_center[0]
distance_to_center_y = distances_from_center[1]  # Choisissez la distance appropriée selon votre configuration

# Calcul de la vitesse orbitale pour les composantes x et y
vitesse_orbitale_x = np.sqrt(G * circles[0].mass / distance_to_center_x)
vitesse_orbitale_y = np.sqrt(G * circles[0].mass / distance_to_center_y)

for i, (x, y) in enumerate(circles_coordinates):
    # Calculer la distance entre le cercle actuel et le cercle central
    distance_to_center = np.sqrt((x - center_x)**2 + (y - center_y)**2)

    # Calculer la vitesse orbitale nécessaire pour une orbite circulaire
    vitesse_orbitale = np.sqrt(G * circles[0].mass / distance_to_center)

    # Calculer la direction de l'orbite (perpendiculaire au vecteur position)
    direction_orbite = np.array([y - center_y, -(x - center_x)]) / distance_to_center

    # Appliquer la vitesse orbitale dans la direction de l'orbite
    vitesse_orbitale_vector = vitesse_orbitale * direction_orbite

    cercle = Cercle(screen, x, y, 10, 100, vitesse_orbitale_vector)
    circles.append(cercle)


fps = 60
clock = pygame.time.Clock()

running = True
while running:
    dt = 50000  # Essayez avec une valeur plus petite
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                running = False

        elif event.type == pygame.VIDEORESIZE:
            # Gérer le redimensionnement de la fenêtre
            width, height = event.size
            screen = pygame.display.set_mode((width, height), pygame.RESIZABLE)

    screen.fill((0, 0, 0))

    for i, circle in enumerate(circles):

        force_x, force_y = 0, 0
        for j, autre_circle in enumerate(circles):
            if i != j:
                delta_force_x, delta_force_y = calculer_vecteur_force(circle, autre_circle)
                force_x += delta_force_x
                force_y += delta_force_y

                distance_between_circles = np.sqrt((circle.x - autre_circle.x) ** 2 + (circle.y - autre_circle.y) ** 2)
                sum_of_radii = circle.radius + autre_circle.radius +10
                if distance_between_circles < sum_of_radii:
                    # Les cercles sont en collision, appelez la fonction de rebond
                    collision_normal = np.array([autre_circle.x - circle.x, autre_circle.y - circle.y])
                    collision_normal /= np.linalg.norm(collision_normal)
                    collision_rebond(circle, autre_circle, collision_normal)

        acceleration_x = force_x / circle.mass
        acceleration_y = force_y / circle.mass

        # Mettez à jour les coordonnées du cercle avec dt
        circle.x += circle.velocity[0] * dt
        circle.y += circle.velocity[1] * dt

        circle.velocity[0] += acceleration_x * dt
        circle.velocity[1] += acceleration_y * dt

        # Utilisez les nouvelles coordonnées pour dessiner le cercle
        circle.draw()

    pygame.display.flip()
    clock.tick(fps)  # Limiter les FPS pour une simulation stable

pygame.quit()
sys.exit()
