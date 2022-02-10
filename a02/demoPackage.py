from wordcloud import WordCloud
 
text = open("demoPackage.txt").read() 
wordcloud = WordCloud(max_font_size=40).generate(text)
wordcloud.to_file("demoPackage.png")